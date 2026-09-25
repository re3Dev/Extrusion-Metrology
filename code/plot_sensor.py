import serial
from serial import SerialException
import matplotlib.pyplot as plt
import numpy as np
from threading import Event, Lock, Thread
from time import perf_counter

plt.style.use("dark_background")


# --------------------------------------------------
# SETTINGS
# --------------------------------------------------

PORT = "COM3"       # CHANGE THIS
BAUD = 115200

EXPECTED_PIXELS = 1500
FRAME_MAGIC = b"TCD1"
FRAME_HEADER_SIZE = 10
FRAME_SIZE = FRAME_HEADER_SIZE + EXPECTED_PIXELS * 2
SMOOTHING_WINDOW = 21
NOISE_FLOOR = 8
REFERENCE_FRAMES = 20
NOISE_MULTIPLIER = 3.0
HISTORY_FRAMES = 120
HISTORY_DOWNSAMPLE = 3
EDGE_THRESHOLD = 10
MAX_EDGE_GAP = 4
MIN_OBJECT_WIDTH = 5
GUI_INTERVAL_MS = 20
DISPLAY_SCALE = 250.0


# --------------------------------------------------
# SERIAL
# --------------------------------------------------

ser = serial.Serial(
    PORT,
    BAUD,
    timeout=0.1
)

print("Connected to", PORT)
frame_lock = Lock()
stop_event = Event()
latest_packet = None
latest_sequence = 0
connection_error = None


# --------------------------------------------------
# GRAPH
# --------------------------------------------------

x = np.arange(EXPECTED_PIXELS)
current_exposure = None
reference = None
reference_frames = []
noise_threshold = np.full(EXPECTED_PIXELS, NOISE_FLOOR, dtype=float)
displayed_change = np.zeros(EXPECTED_PIXELS)
history = np.zeros((
    HISTORY_FRAMES,
    EXPECTED_PIXELS // HISTORY_DOWNSAMPLE
))
processed_sequence = 0
last_frame_time = None
display_fps = 0.0

fig, (profile_ax, history_ax) = plt.subplots(
    2,
    1,
    figsize=(13, 8),
    height_ratios=(1, 2),
    sharex=True
)
fig.patch.set_facecolor("#0d1117")

for axis in (profile_ax, history_ax):
    axis.set_facecolor("#161b22")
    axis.tick_params(colors="#c9d1d9")
    axis.xaxis.label.set_color("#c9d1d9")
    axis.yaxis.label.set_color("#c9d1d9")
    axis.title.set_color("#f0f6fc")

    for spine in axis.spines.values():
        spine.set_color("#30363d")

profile_line, = profile_ax.plot(
    x,
    displayed_change,
    linewidth=2,
    color="#e4572e"
)
profile_ax.axhline(
    EDGE_THRESHOLD,
    color="#777777",
    linestyle="--",
    linewidth=1
)
left_edge_line = profile_ax.axvline(
    0,
    color="#00d4ff",
    linewidth=2,
    visible=False
)
right_edge_line = profile_ax.axvline(
    0,
    color="#7cff6b",
    linewidth=2,
    visible=False
)

profile_ax.set_title("Live Object Profile")
profile_ax.set_ylabel("Signal")
profile_ax.set_xlim(0, EXPECTED_PIXELS - 1)
profile_ax.set_ylim(0, DISPLAY_SCALE)
profile_ax.grid(True, color="#30363d", alpha=0.7)

history_image = history_ax.imshow(
    history,
    aspect="auto",
    origin="upper",
    extent=(0, EXPECTED_PIXELS - 1, HISTORY_FRAMES, 0),
    cmap="inferno",
    vmin=0,
    vmax=DISPLAY_SCALE,
    interpolation="nearest"
)
history_ax.set_title("Sweep History (Newest Frame at Top)")
history_ax.set_xlabel("Active pixel")
history_ax.set_ylabel("Frames ago")
colorbar = fig.colorbar(history_image, ax=history_ax, pad=0.01)
colorbar.set_label("Object signal")
colorbar.ax.tick_params(colors="#c9d1d9")
colorbar.ax.yaxis.label.set_color("#c9d1d9")
colorbar.outline.set_edgecolor("#30363d")

status_text = profile_ax.text(
    0.99,
    0.93,
    "Learning scene",
    transform=profile_ax.transAxes,
    horizontalalignment="right",
    verticalalignment="top",
    color="#f0f6fc",
    bbox={
        "facecolor": "#0d1117",
        "edgecolor": "#30363d",
        "alpha": 0.85,
        "pad": 4
    }
)

fig.tight_layout()


# --------------------------------------------------
# UPDATE
# --------------------------------------------------

def smooth_profile(values):

    radius = SMOOTHING_WINDOW // 2
    padded = np.pad(values, radius, mode="edge")
    kernel = np.full(SMOOTHING_WINDOW, 1.0 / SMOOTHING_WINDOW)
    return np.convolve(padded, kernel, mode="valid")


def find_object_edges(signal):

    active = signal >= EDGE_THRESHOLD
    inactive = np.flatnonzero(~active)

    for start, end in zip(inactive[:-1], inactive[1:]):
        if 1 < end - start <= MAX_EDGE_GAP + 1:
            active[start + 1:end] = True

    transitions = np.diff(
        np.pad(active.astype(np.int8), (1, 1))
    )
    starts = np.flatnonzero(transitions == 1)
    ends = np.flatnonzero(transitions == -1) - 1
    candidates = [
        (left, right)
        for left, right in zip(starts, ends)
        if right - left + 1 >= MIN_OBJECT_WIDTH
    ]

    if not candidates:
        return None

    return max(
        candidates,
        key=lambda edges: np.sum(signal[edges[0]:edges[1] + 1])
    )


def reset_reference(event=None):

    global reference, reference_frames, noise_threshold
    global displayed_change, history

    if event is None or event.key.lower() == "r":
        reference = None
        reference_frames = []
        noise_threshold.fill(NOISE_FLOOR)
        displayed_change.fill(0)
        history.fill(0)
        profile_line.set_ydata(displayed_change)
        history_image.set_data(history)
        left_edge_line.set_visible(False)
        right_edge_line.set_visible(False)
        status_text.set_text("Learning scene")


fig.canvas.mpl_connect("key_press_event", reset_reference)


def read_sensor_frames():

    global latest_packet, latest_sequence, connection_error
    buffer = bytearray()

    while not stop_event.is_set():
        try:
            chunk = ser.read(max(ser.in_waiting, 1))
        except SerialException as error:
            connection_error = str(error)
            return

        if not chunk:
            continue

        buffer.extend(chunk)

        while True:
            magic_index = buffer.find(FRAME_MAGIC)
            if magic_index < 0:
                del buffer[:-3]
                break

            if magic_index:
                del buffer[:magic_index]

            if len(buffer) < FRAME_SIZE:
                break

            exposure = int.from_bytes(buffer[4:8], "little")
            pixel_count = int.from_bytes(buffer[8:10], "little")
            if pixel_count != EXPECTED_PIXELS:
                del buffer[0]
                continue

            values = np.frombuffer(
                buffer,
                dtype="<u2",
                count=EXPECTED_PIXELS,
                offset=FRAME_HEADER_SIZE
            ).copy()
            del buffer[:FRAME_SIZE]

            with frame_lock:
                latest_packet = (exposure, values, perf_counter())
                latest_sequence += 1


def update_display():

    global current_exposure, reference, reference_frames, noise_threshold
    global displayed_change, history, processed_sequence
    global last_frame_time, display_fps

    if connection_error is not None:
        status_text.set_text(f"Serial disconnected: {connection_error}")
        fig.canvas.draw_idle()
        return False

    with frame_lock:
        if latest_packet is None or latest_sequence == processed_sequence:
            return True

        exposure, latest, received_at = latest_packet
        processed_sequence = latest_sequence

    if last_frame_time is not None:
        instantaneous_fps = 1.0 / max(received_at - last_frame_time, 0.001)
        display_fps += 0.2 * (instantaneous_fps - display_fps)
    last_frame_time = received_at

    if exposure != current_exposure:
        current_exposure = exposure
        reset_reference()
        profile_ax.set_title(
            "Live Object Profile - "
            f"Exposure {current_exposure} us"
        )

    smoothed = smooth_profile(latest.astype(float))
    centered = smoothed - np.median(smoothed)

    if reference is None:
        reference_frames.append(centered)
        status_text.set_text(
            f"Learning scene {len(reference_frames)}/{REFERENCE_FRAMES}"
        )

        if len(reference_frames) == REFERENCE_FRAMES:
            calibration = np.asarray(reference_frames)
            reference = np.median(calibration, axis=0)
            median_deviation = np.median(
                np.abs(calibration - reference),
                axis=0
            )
            noise_threshold = np.maximum(
                NOISE_FLOOR,
                NOISE_MULTIPLIER * 1.4826 * median_deviation
            )
            reference_frames = []
            status_text.set_text(f"Reference locked | {display_fps:.1f} FPS")
    else:
        residual = np.abs(centered - reference)
        displayed_change = np.maximum(0, residual - noise_threshold)

        history[1:] = history[:-1].copy()
        history[0] = displayed_change.reshape(
            -1,
            HISTORY_DOWNSAMPLE
        ).max(axis=1)

        profile_line.set_ydata(displayed_change)
        history_image.set_data(history)

        edges = find_object_edges(displayed_change)
        if edges is not None:
            left_edge, right_edge = edges
            region = np.arange(left_edge, right_edge + 1)
            center = int(np.average(
                region,
                weights=displayed_change[region]
            ))
            width = right_edge - left_edge + 1
            left_edge_line.set_xdata([left_edge, left_edge])
            right_edge_line.set_xdata([right_edge, right_edge])
            left_edge_line.set_visible(True)
            right_edge_line.set_visible(True)
            status_text.set_text(
                f"Object: {left_edge} | {center} | {right_edge}  "
                f"width {width} px | {display_fps:.1f} FPS"
            )
        else:
            left_edge_line.set_visible(False)
            right_edge_line.set_visible(False)
            status_text.set_text(f"No object | {display_fps:.1f} FPS")

    fig.canvas.draw_idle()
    return True


def close_plot(event):

    stop_event.set()


fig.canvas.mpl_connect("close_event", close_plot)

reader_thread = Thread(target=read_sensor_frames, daemon=True)
reader_thread.start()

timer = fig.canvas.new_timer(interval=GUI_INTERVAL_MS)
timer.add_callback(update_display)
timer.start()

plt.show()

stop_event.set()
reader_thread.join(timeout=0.5)
ser.close()