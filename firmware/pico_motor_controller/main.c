#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"

// ============================================================
// Pin configuration
// ============================================================

// Motor PWM outputs
#define FR_PWM_PIN 18
#define BR_PWM_PIN 19
#define FL_PWM_PIN 20
#define BL_PWM_PIN 21

// Status LED
#define LED_PIN 25
#define LED_TICK_MS 250
#define LED_OFF_TICKS 3

// Encoder pins
#define FR_ENC_A 6
#define FR_ENC_B 7

#define BR_ENC_A 8
#define BR_ENC_B 9

#define FL_ENC_A 10
#define FL_ENC_B 11

#define BL_ENC_A 12
#define BL_ENC_B 13

// Gripper analog sensors
// Purple (hinge) -> GP27 = ADC1, blue (anvil) -> GP26 = ADC0.
// Raw 12-bit ADC counts over serial; Jetson converts to volts.
#define GRIP_HINGE_ADC 1
#define GRIP_ANVIL_ADC 0
#define GRIP_ADC_PIN_HINGE 27
#define GRIP_ADC_PIN_ANVIL 26
#define GRIP_ADC_SAMPLES 8
#define GRIP_REPORT_INTERVAL_US 100000ULL  // 10 Hz

// ============================================================
// Motor PWM configuration
// ============================================================

#define PWM_FREQ_HZ 50
#define PWM_WRAP 19999

#define MOTOR_CENTER_US 1500
#define MOTOR_RANGE_US 400

// Failsafe timeout
#define MOTOR_FAILSAFE_MS 200

// ============================================================
// Encoder configuration
// ============================================================

typedef struct {
    uint pin_a;
    uint pin_b;

    volatile int32_t count;
    volatile uint8_t state;

    int direction;
} encoder_t;

static encoder_t enc_fr = {
    .pin_a = FR_ENC_A,
    .pin_b = FR_ENC_B,
    .count = 0,
    .state = 0,
    .direction = 1
};

static encoder_t enc_br = {
    .pin_a = BR_ENC_A,
    .pin_b = BR_ENC_B,
    .count = 0,
    .state = 0,
    .direction = 1
};

static encoder_t enc_fl = {
    .pin_a = FL_ENC_A,
    .pin_b = FL_ENC_B,
    .count = 0,
    .state = 0,
    .direction = 1
};

static encoder_t enc_bl = {
    .pin_a = BL_ENC_A,
    .pin_b = BL_ENC_B,
    .count = 0,
    .state = 0,
    .direction = 1
};

// Quadrature transition table.
// Index = (old_state << 2) | new_state
//
// Valid transitions:
//
// 00 -> 01 : +1
// 00 -> 10 : -1
// 01 -> 00 : -1
// 01 -> 11 : +1
// 10 -> 00 : +1
// 10 -> 11 : -1
// 11 -> 01 : -1
// 11 -> 10 : +1
//
// Invalid transitions produce 0.
static const int8_t quadrature_table[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

// ============================================================
// Motor PWM
// ============================================================

static float clamp_float(float value, float min_value, float max_value)
{
    if (value < min_value)
        return min_value;

    if (value > max_value)
        return max_value;

    return value;
}

static void pwm_init_motor_pin(uint gpio)
{
    gpio_set_function(gpio, GPIO_FUNC_PWM);

    uint slice = pwm_gpio_to_slice_num(gpio);
    uint channel = pwm_gpio_to_channel(gpio);

    // 125 MHz system clock / 125 = 1 MHz
    // Therefore each PWM tick = 1 us.
    pwm_config config = pwm_get_default_config();

    pwm_config_set_clkdiv(&config, 125.0f);
    pwm_config_set_wrap(&config, PWM_WRAP);

    pwm_init(slice, &config, false);

    // Start centered at 1500 us.
    pwm_set_chan_level(slice, channel, MOTOR_CENTER_US);
}

static void motor_pwm_init(void)
{
    pwm_init_motor_pin(FL_PWM_PIN);
    pwm_init_motor_pin(FR_PWM_PIN);
    pwm_init_motor_pin(BL_PWM_PIN);
    pwm_init_motor_pin(BR_PWM_PIN);

    // IMPORTANT:
    //
    // GP18/GP19 use PWM slice 9.
    // GP20/GP21 use PWM slice 10.
    //
    // Both slices must be enabled.
    uint slice_right = pwm_gpio_to_slice_num(FR_PWM_PIN);
    uint slice_left  = pwm_gpio_to_slice_num(FL_PWM_PIN);

    pwm_set_enabled(slice_right, true);
    pwm_set_enabled(slice_left, true);
}

static void motor_set(uint gpio, float value)
{
    value = clamp_float(value, -1.0f, 1.0f);

    int pulse_us =
        MOTOR_CENTER_US +
        (int)(value * MOTOR_RANGE_US);

    uint slice = pwm_gpio_to_slice_num(gpio);
    uint channel = pwm_gpio_to_channel(gpio);

    pwm_set_chan_level(slice, channel, pulse_us);
}

static void motor_stop_all(void)
{
    motor_set(FL_PWM_PIN, 0.0f);
    motor_set(FR_PWM_PIN, 0.0f);
    motor_set(BL_PWM_PIN, 0.0f);
    motor_set(BR_PWM_PIN, 0.0f);
}

// ============================================================
// Encoder handling
// ============================================================

static inline uint8_t encoder_read_state(const encoder_t *enc)
{
    uint8_t a = gpio_get(enc->pin_a);
    uint8_t b = gpio_get(enc->pin_b);

    return (a << 1) | b;
}

static void encoder_update(encoder_t *enc)
{
    uint8_t new_state = encoder_read_state(enc);

    uint8_t index =
        (enc->state << 2) |
        new_state;

    int8_t delta = quadrature_table[index];

    enc->count += delta * enc->direction;
    enc->state = new_state;
}

static void encoder_gpio_callback(
    uint gpio,
    uint32_t events)
{
    (void)events;

    if (gpio == enc_fr.pin_a || gpio == enc_fr.pin_b) {
        encoder_update(&enc_fr);
    }
    else if (gpio == enc_br.pin_a || gpio == enc_br.pin_b) {
        encoder_update(&enc_br);
    }
    else if (gpio == enc_fl.pin_a || gpio == enc_fl.pin_b) {
        encoder_update(&enc_fl);
    }
    else if (gpio == enc_bl.pin_a || gpio == enc_bl.pin_b) {
        encoder_update(&enc_bl);
    }
}

static void encoder_init(encoder_t *enc)
{
    gpio_init(enc->pin_a);
    gpio_init(enc->pin_b);

    gpio_set_dir(enc->pin_a, GPIO_IN);
    gpio_set_dir(enc->pin_b, GPIO_IN);

    gpio_pull_up(enc->pin_a);
    gpio_pull_up(enc->pin_b);

    // Capture the initial quadrature state before enabling IRQs.
    enc->state = encoder_read_state(enc);
}

static void encoders_init(void)
{
    encoder_init(&enc_fr);
    encoder_init(&enc_br);
    encoder_init(&enc_fl);
    encoder_init(&enc_bl);

    gpio_set_irq_enabled_with_callback(
        FR_ENC_A,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true,
        &encoder_gpio_callback
    );

    gpio_set_irq_enabled(
        FR_ENC_B,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true
    );

    gpio_set_irq_enabled(
        BR_ENC_A,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true
    );

    gpio_set_irq_enabled(
        BR_ENC_B,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true
    );

    gpio_set_irq_enabled(
        FL_ENC_A,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true
    );

    gpio_set_irq_enabled(
        FL_ENC_B,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true
    );

    gpio_set_irq_enabled(
        BL_ENC_A,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true
    );

    gpio_set_irq_enabled(
        BL_ENC_B,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true
    );
}

// ============================================================
// Gripper analog sensors
// ============================================================

static uint32_t grip_read_adc(uint adc_input)
{
    adc_select_input(adc_input);

    uint32_t sum = 0;

    for (uint i = 0; i < GRIP_ADC_SAMPLES; i++) {
        sum += adc_read();
    }

    return sum / GRIP_ADC_SAMPLES;
}

static void gripper_init(void)
{
    adc_init();

    adc_gpio_init(GRIP_ADC_PIN_HINGE);
    adc_gpio_init(GRIP_ADC_PIN_ANVIL);
}

static void print_gripper(void)
{
    uint32_t hinge = grip_read_adc(GRIP_HINGE_ADC);
    uint32_t anvil = grip_read_adc(GRIP_ANVIL_ADC);

    uint64_t now_us = time_us_64();

    printf("GR %lu %lu %llu\n",
        (unsigned long)hinge,
        (unsigned long)anvil,
        (unsigned long long)now_us);
}

// ============================================================
// Serial command handling
// ============================================================

static char serial_line[128];
static size_t serial_line_len = 0;

static uint64_t last_motor_command_us = 0;

static void process_motor_command(const char *line)
{
    float fl;
    float fr;
    float bl;
    float br;

    if (sscanf(line, "%f,%f,%f,%f", &fl, &fr, &bl, &br) == 4) {

        motor_set(FL_PWM_PIN, fl);
        motor_set(FR_PWM_PIN, fr);
        motor_set(BL_PWM_PIN, bl);
        motor_set(BR_PWM_PIN, br);

        last_motor_command_us = time_us_64();
    }
}

static bool led_heartbeat_callback(repeating_timer_t *timer)
{
    (void)timer;

    static bool led_is_on = true;
    static uint8_t off_ticks = 0;

    if (led_is_on) {
        led_is_on = false;
        off_ticks = 0;
        gpio_put(LED_PIN, 0);
    }
    else if (++off_ticks >= LED_OFF_TICKS) {
        led_is_on = true;
        gpio_put(LED_PIN, 1);
    }

    return true;
}

static void process_serial_line(const char *line)
{
    if (strlen(line) == 0)
        return;

    process_motor_command(line);
}

static void serial_poll(void)
{
    int ch;

    while ((ch = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {

        if (ch == '\r')
            continue;

        if (ch == '\n') {

            serial_line[serial_line_len] = '\0';

            process_serial_line(serial_line);

            serial_line_len = 0;
            continue;
        }

        if (serial_line_len < sizeof(serial_line) - 1) {
            serial_line[serial_line_len++] = (char)ch;
        }
        else {
            // Overflow: discard the current line.
            serial_line_len = 0;
        }
    }
}

// ============================================================
// Encoder reporting
// ============================================================

static void print_encoder_counts(void)
{
    int32_t fl = enc_fl.count;
    int32_t fr = enc_fr.count;
    int32_t bl = enc_bl.count;
    int32_t br = enc_br.count;

    // Report a Pico-clock timestamp with each sample: the host computes
    // rpm from Pico-time deltas, so host timer jitter and burst arrival
    // (2 reports one tick, none the next) can't alias into rpm spikes.
    uint64_t now_us = time_us_64();

    printf("FL %ld %llu\n", (long)fl, (unsigned long long)now_us);
    printf("FR %ld %llu\n", (long)fr, (unsigned long long)now_us);
    printf("BL %ld %llu\n", (long)bl, (unsigned long long)now_us);
    printf("BR %ld %llu\n", (long)br, (unsigned long long)now_us);
}

// ============================================================
// Main
// ============================================================

int main(void)
{
    stdio_init_all();

    sleep_ms(500);

    // LED
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);

    repeating_timer_t led_timer;
    if (!add_repeating_timer_ms(
            LED_TICK_MS,
            led_heartbeat_callback,
            NULL,
            &led_timer)) {
        printf("Failed to start LED heartbeat timer\n");
    }

    // Motors
    motor_pwm_init();
    motor_stop_all();

    // Encoders
    encoders_init();

    // Gripper sensors
    gripper_init();

    last_motor_command_us = time_us_64();

    uint64_t last_encoder_report_us = time_us_64();
    uint64_t last_gripper_report_us = time_us_64();

    while (true) {

        // ----------------------------------------------------
        // Serial input
        // ----------------------------------------------------

        serial_poll();

        // ----------------------------------------------------
        // Motor failsafe
        // ----------------------------------------------------

        uint64_t now_us = time_us_64();

        if ((now_us - last_motor_command_us) >
            (MOTOR_FAILSAFE_MS * 1000ULL)) {

            motor_stop_all();
        }

        // ----------------------------------------------------
        // Encoder reporting
        // ----------------------------------------------------

        // 100 Hz (10 ms): faster measurement for the Jetson-side speed
        // PID. Jetson cmd_vel_serial must drain at >= this rate or the
        // extra reports are wasted (last-report-wins per tick).
        if ((now_us - last_encoder_report_us) >= 10000ULL) {

            last_encoder_report_us = now_us;

            print_encoder_counts();
        }

        // ----------------------------------------------------
        // Gripper reporting (10 Hz)
        // ----------------------------------------------------

        if ((now_us - last_gripper_report_us) >=
            GRIP_REPORT_INTERVAL_US) {

            last_gripper_report_us = now_us;

            print_gripper();
        }

        // Small sleep to avoid spinning unnecessarily.
        tight_loop_contents();
    }

    return 0;
}

