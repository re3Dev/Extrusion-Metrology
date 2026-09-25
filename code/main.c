#include <stdint.h>
#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/adc.h"

#define PIN_PHIM 2
#define PIN_ICG  3
#define PIN_SH   4
#define PIN_LED  5

#define ADC_GPIO    26
#define ADC_CHANNEL 0

#define CCD_PERIODS        1546
#define ACTIVE_PIXEL_START 32
#define TEST_PIXELS        1500

#define PHIM_HALF_PERIOD_US 1
#define ICG_TO_SH_CYCLES    63
#define EXPOSURE_COUNT       1
#define EXPOSURE_HOLD_FRAMES 1
#define SAMPLE_PHASE         0
#define SH_PERIOD_US         12
#define SH_HIGH_US           4
#define SH_LOW_US            (SH_PERIOD_US - SH_HIGH_US)
#define SH_HIGH_CLOCKS       (SH_HIGH_US / (2 * PHIM_HALF_PERIOD_US))
#define SH_LOW_CLOCKS        (SH_LOW_US / (2 * PHIM_HALF_PERIOD_US))
#define CLEAR_TIME_US        50000

static uint16_t profile[TEST_PIXELS];
static const uint32_t exposure_us[EXPOSURE_COUNT] = {
    100
};

static void setup_sensor_gpio(void)
{
    const uint pins[] = {
        PIN_PHIM,
        PIN_ICG,
        PIN_SH,
        PIN_LED
    };

    for (uint i = 0; i < count_of(pins); i++)
    {
        gpio_init(pins[i]);
        gpio_set_dir(pins[i], GPIO_OUT);
    }

    gpio_put(PIN_PHIM, 1);
    gpio_put(PIN_ICG, 1);
    gpio_put(PIN_SH, 0);
    gpio_put(PIN_LED, 1);
}

static void setup_adc(void)
{
    adc_init();
    adc_gpio_init(ADC_GPIO);
    adc_select_input(ADC_CHANNEL);
    adc_fifo_setup(false, false, 1, false, false);
    adc_set_clkdiv(0.0f);
}

static void clock_phim_cycle(void)
{
    gpio_put(PIN_PHIM, 0);
    busy_wait_us_32(PHIM_HALF_PERIOD_US);

    gpio_put(PIN_PHIM, 1);
    busy_wait_us_32(PHIM_HALF_PERIOD_US);
}

static void start_adc_conversion(void)
{
    hw_set_bits(
        &adc_hw->cs,
        ADC_CS_START_ONCE_BITS
    );
}

static uint16_t finish_adc_conversion(void)
{
    while (!(adc_hw->cs & ADC_CS_READY_BITS))
    {
        tight_loop_contents();
    }

    return (uint16_t) adc_hw->result;
}

static uint16_t clock_and_sample_period(uint sample_phase)
{
    gpio_put(PIN_PHIM, 0);

    if (sample_phase == 0)
    {
        start_adc_conversion();
    }

    busy_wait_us_32(PHIM_HALF_PERIOD_US);

    gpio_put(PIN_PHIM, 1);

    if (sample_phase == 1)
    {
        start_adc_conversion();
    }

    busy_wait_us_32(PHIM_HALF_PERIOD_US);

    gpio_put(PIN_PHIM, 0);

    if (sample_phase == 2)
    {
        start_adc_conversion();
    }

    busy_wait_us_32(PHIM_HALF_PERIOD_US);

    gpio_put(PIN_PHIM, 1);

    if (sample_phase == 3)
    {
        start_adc_conversion();
    }

    busy_wait_us_32(PHIM_HALF_PERIOD_US);

    return finish_adc_conversion();
}

static void transfer_sensor_line(void)
{
    gpio_put(PIN_PHIM, 1);
    gpio_put(PIN_ICG, 0);

    // 63 cycles is approximately 500 ns at the default 125 MHz sys clock.
    busy_wait_at_least_cycles(ICG_TO_SH_CYCLES);

    gpio_put(PIN_SH, 1);

    for (uint cycle = 0;
         cycle < SH_HIGH_CLOCKS;
         cycle++)
    {
        clock_phim_cycle();
    }

    gpio_put(PIN_SH, 0);
    clock_phim_cycle();
    clock_phim_cycle();
    clock_phim_cycle();

    // clock_phim_cycle() returns with PHIM high, as required for ICG rising.
    gpio_put(PIN_ICG, 1);
    clock_phim_cycle();
}

static void clear_sensor_charge(void)
{
    uint32_t shutter_cycles =
        CLEAR_TIME_US / SH_PERIOD_US;

    gpio_put(PIN_ICG, 1);

    for (uint32_t pulse = 0;
         pulse < shutter_cycles;
         pulse++)
    {
        gpio_put(PIN_SH, 1);

        for (uint cycle = 0;
             cycle < SH_HIGH_CLOCKS;
             cycle++)
        {
            clock_phim_cycle();
        }

        gpio_put(PIN_SH, 0);

        for (uint cycle = 0;
             cycle < SH_LOW_CLOCKS;
             cycle++)
        {
            clock_phim_cycle();
        }
    }
}

static void capture_line(
    uint sample_phase,
    uint32_t integration_us
)
{
    // Repeated SH pulses clear charge while ICG remains high.
    clear_sensor_charge();

    // The deliberately longer SH-to-SH gap is the selected TINT interval.
    uint32_t integration_clocks =
        integration_us
        / (2 * PHIM_HALF_PERIOD_US);

    for (uint32_t cycle = 0;
         cycle < integration_clocks;
         cycle++)
    {
        clock_phim_cycle();
    }

    // End TINT and transfer the integrated line. This SH pulse has the same
    // width as all electronic-shutter pulses.
    transfer_sensor_line();

    for (int period = 0; period < CCD_PERIODS; period++)
    {
        // Continue the regular SH cycle during readout with ICG high.
        if ((period % 3) == 0)
        {
            gpio_put(PIN_SH, 1);
        }
        else
        {
            gpio_put(PIN_SH, 0);
        }

        uint16_t sample =
            clock_and_sample_period(sample_phase);
        int active_pixel = period - ACTIVE_PIXEL_START;

        if (active_pixel >= 0 && active_pixel < TEST_PIXELS)
        {
            profile[active_pixel] = sample;
        }
    }

    gpio_put(PIN_SH, 0);
}

static void print_profile(uint32_t integration_us)
{
    static const uint8_t frame_magic[4] = {
        'T', 'C', 'D', '1'
    };
    const uint16_t pixel_count = TEST_PIXELS;

    fwrite(frame_magic, sizeof(frame_magic), 1, stdout);
    fwrite(&integration_us, sizeof(integration_us), 1, stdout);
    fwrite(&pixel_count, sizeof(pixel_count), 1, stdout);
    fwrite(profile, sizeof(profile), 1, stdout);
    fflush(stdout);
}

int main(void)
{
    stdio_init_all();
    stdio_set_translate_crlf(&stdio_usb, false);
    sleep_ms(2000);

    setup_sensor_gpio();
    setup_adc();

    uint exposure_index = 0;
    uint exposure_frame = 0;

    while (true)
    {
        uint32_t integration_us =
            exposure_us[exposure_index];

        capture_line(
            SAMPLE_PHASE,
            integration_us
        );
        print_profile(integration_us);

        exposure_frame++;

        if (exposure_frame == EXPOSURE_HOLD_FRAMES)
        {
            exposure_frame = 0;
            exposure_index =
                (exposure_index + 1)
                % EXPOSURE_COUNT;
        }

        sleep_ms(20);
    }
}