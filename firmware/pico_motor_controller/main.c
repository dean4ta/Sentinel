#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"

// ============================================================
// Configuration
// ============================================================

#define PWM_FREQ            50
#define PWM_PERIOD_US       20000
#define NEUTRAL_US          1500
#define FAILSAFE_MS         200

#define REPORT_MS           20
#define MAX_LINE_LENGTH     80

#define LED_PIN             25

// Direct-drive motor PWM pins
#define FL_PWM_PIN          20
#define FR_PWM_PIN          18
#define BL_PWM_PIN          21
#define BR_PWM_PIN          19

// Encoder pins
#define FR_ENC_A            6
#define FR_ENC_B            7

#define BR_ENC_A            8
#define BR_ENC_B            9

#define FL_ENC_A            10
#define FL_ENC_B            11

#define BL_ENC_A            12
#define BL_ENC_B            13

// Encoder direction multipliers
#define FL_ENCODER_DIRECTION    1
#define FR_ENCODER_DIRECTION    1
#define BL_ENCODER_DIRECTION    1
#define BR_ENCODER_DIRECTION    1


// ============================================================
// Encoder state
// ============================================================

typedef struct {
    uint8_t pin_a;
    uint8_t pin_b;

    int direction;

    volatile int32_t count;
    volatile uint8_t state;
} Encoder;

static Encoder enc_fl = {
    .pin_a = FL_ENC_A,
    .pin_b = FL_ENC_B,
    .direction = FL_ENCODER_DIRECTION,
    .count = 0,
    .state = 0
};

static Encoder enc_fr = {
    .pin_a = FR_ENC_A,
    .pin_b = FR_ENC_B,
    .direction = FR_ENCODER_DIRECTION,
    .count = 0,
    .state = 0
};

static Encoder enc_bl = {
    .pin_a = BL_ENC_A,
    .pin_b = BL_ENC_B,
    .direction = BL_ENCODER_DIRECTION,
    .count = 0,
    .state = 0
};

static Encoder enc_br = {
    .pin_a = BR_ENC_A,
    .pin_b = BR_ENC_B,
    .direction = BR_ENCODER_DIRECTION,
    .count = 0,
    .state = 0
};


// ============================================================
// Quadrature transition table
//
// state = (A << 1) | B
//
//        current
// prev   0   1   2   3
//  0     0  -1  +1   0
//  1    +1   0   0  -1
//  2    -1   0   0  +1
//  3     0  +1  -1   0
// ============================================================

static const int8_t transition_table[4][4] = {
    {  0, -1,  1,  0 },
    {  1,  0,  0, -1 },
    { -1,  0,  0,  1 },
    {  0,  1, -1,  0 }
};


// ============================================================
// PWM
// ============================================================

static uint pwm_slice;

static void pwm_init_motor_pin(uint pin)
{
    gpio_set_function(pin, GPIO_FUNC_PWM);

    uint slice = pwm_gpio_to_slice_num(pin);

    pwm_config config = pwm_get_default_config();

    /*
     * System clock is normally 125 MHz.
     *
     * Divide by 125 -> 1 MHz PWM counter.
     *
     * Therefore:
     *   1 counter tick = 1 us
     *   20000 counts = 20 ms
     *   50 Hz
     */
    pwm_config_set_clkdiv(&config, 125.0f);
    pwm_config_set_wrap(&config, PWM_PERIOD_US - 1);

    pwm_init(slice, &config, false);

    pwm_set_gpio_level(pin, NEUTRAL_US);

    if (slice != pwm_slice) {
        // All four motor pins should share the same PWM slice.
        // This is only a sanity check during development.
    }
}

static void motor_pwm_init(void)
{
    pwm_slice = pwm_gpio_to_slice_num(FR_PWM_PIN);

    pwm_init_motor_pin(FL_PWM_PIN);
    pwm_init_motor_pin(FR_PWM_PIN);
    pwm_init_motor_pin(BL_PWM_PIN);
    pwm_init_motor_pin(BR_PWM_PIN);

    pwm_set_enabled(pwm_slice, true);
}


static void set_pulse(uint pin, uint16_t us)
{
    if (us > PWM_PERIOD_US - 1) {
        us = PWM_PERIOD_US - 1;
    }

    pwm_set_gpio_level(pin, us);
}


static int float_to_pulse(float v)
{
    if (v < -1.0f)
        v = -1.0f;

    if (v > 1.0f)
        v = 1.0f;

    return NEUTRAL_US + (int)(v * 400.0f);
}


static void neutral_all(void)
{
    set_pulse(FL_PWM_PIN, NEUTRAL_US);
    set_pulse(FR_PWM_PIN, NEUTRAL_US);
    set_pulse(BL_PWM_PIN, NEUTRAL_US);
    set_pulse(BR_PWM_PIN, NEUTRAL_US);
}


// ============================================================
// Encoder initialization
// ============================================================

static inline uint8_t read_encoder_state(const Encoder *enc)
{
    uint8_t a = gpio_get(enc->pin_a);
    uint8_t b = gpio_get(enc->pin_b);

    return (a << 1) | b;
}


static void encoder_init(Encoder *enc)
{
    gpio_init(enc->pin_a);
    gpio_set_dir(enc->pin_a, GPIO_IN);
    gpio_pull_up(enc->pin_a);

    gpio_init(enc->pin_b);
    gpio_set_dir(enc->pin_b, GPIO_IN);
    gpio_pull_up(enc->pin_b);

    /*
     * IMPORTANT:
     *
     * Capture the actual state BEFORE enabling interrupts.
     * This prevents a startup transition from generating
     * a false count.
     */
    enc->state = read_encoder_state(enc);
    enc->count = 0;
}


// ============================================================
// Encoder IRQ
// ============================================================

static inline void encoder_update(Encoder *enc)
{
    uint8_t current = read_encoder_state(enc);
    uint8_t previous = enc->state;

    int8_t delta = transition_table[previous][current];

    if (delta != 0) {
        enc->count += delta * enc->direction;
    }

    enc->state = current;
}


/*
 * A single GPIO callback handles all encoder pins.
 *
 * We identify which encoder owns the pin that caused the
 * interrupt, then ONLY decode that encoder.
 *
 * This preserves the important behavior of your Python
 * implementation where an FR edge cannot modify FL/BL/BR.
 */
static void encoder_gpio_callback(
    uint gpio,
    uint32_t events
)
{
    (void)events;

    if (gpio == FL_ENC_A || gpio == FL_ENC_B) {
        encoder_update(&enc_fl);
    }
    else if (gpio == FR_ENC_A || gpio == FR_ENC_B) {
        encoder_update(&enc_fr);
    }
    else if (gpio == BL_ENC_A || gpio == BL_ENC_B) {
        encoder_update(&enc_bl);
    }
    else if (gpio == BR_ENC_A || gpio == BR_ENC_B) {
        encoder_update(&enc_br);
    }
}


static void encoder_irq_init(void)
{
    /*
     * Register the callback once, then enable both rising
     * and falling edges on every encoder pin.
     */

    gpio_set_irq_enabled_with_callback(
        FL_ENC_A,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true,
        &encoder_gpio_callback
    );

    gpio_set_irq_enabled(
        FL_ENC_B,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true
    );

    gpio_set_irq_enabled(
        FR_ENC_A,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true
    );

    gpio_set_irq_enabled(
        FR_ENC_B,
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
}


// ============================================================
// LED
// ============================================================

static uint32_t led_on_ms = 100;

static bool led_on_phase = true;
static uint32_t led_phase_start_ms;


static void led_update(uint32_t now)
{
    if (led_on_phase) {

        if ((uint32_t)(now - led_phase_start_ms) >= led_on_ms) {
            gpio_put(LED_PIN, false);

            led_on_phase = false;
            led_phase_start_ms = now;
        }

    } else {

        if ((uint32_t)(now - led_phase_start_ms) >= 200) {
            gpio_put(LED_PIN, true);

            led_on_phase = true;
            led_phase_start_ms = now;
        }
    }
}


// ============================================================
// Serial line handling
// ============================================================

static char current_line[MAX_LINE_LENGTH + 1];
static char newest_line[MAX_LINE_LENGTH + 1];

static uint current_line_length = 0;
static bool newest_line_available = false;


/*
 * Consume every byte currently available.
 *
 * If multiple complete lines arrive before the main loop gets
 * back here, newest_line is overwritten each time.
 *
 * Therefore only the newest complete command is eventually
 * applied.
 */
static void read_serial_newest_only(void)
{
    int c;

    while ((c = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {

        if (c == '\n') {

            current_line[current_line_length] = '\0';

            strcpy(newest_line, current_line);
            newest_line_available = true;

            current_line_length = 0;

        }
        else if (c == '\r') {
            // Ignore CR, matching the Python implementation.
        }
        else {

            if (current_line_length < MAX_LINE_LENGTH) {

                current_line[current_line_length++] = (char)c;

            } else {

                /*
                 * Oversized line.
                 *
                 * Drop the current line until the next newline.
                 */
                current_line_length = 0;
            }
        }
    }
}


// ============================================================
// Parsing
// ============================================================

static char *skip_whitespace(char *p)
{
    while (*p && isspace((unsigned char)*p))
        p++;

    return p;
}


static bool parse_float_field(
    char **cursor,
    float *value
)
{
    char *p = skip_whitespace(*cursor);

    char *end;

    float v = strtof(p, &end);

    if (end == p) {
        return false;
    }

    if (!isfinite(v)) {
        return false;
    }

    *value = v;
    *cursor = end;

    return true;
}


static bool apply_wheel_line(char *line)
{
    float fl, fr, bl, br;

    char *p = line;

    // FL
    if (!parse_float_field(&p, &fl))
        return false;

    p = skip_whitespace(p);

    if (*p != ',')
        return false;

    p++;

    // FR
    if (!parse_float_field(&p, &fr))
        return false;

    p = skip_whitespace(p);

    if (*p != ',')
        return false;

    p++;

    // BL
    if (!parse_float_field(&p, &bl))
        return false;

    p = skip_whitespace(p);

    if (*p != ',')
        return false;

    p++;

    // BR
    if (!parse_float_field(&p, &br))
        return false;

    p = skip_whitespace(p);

    // Must end here.
    if (*p != '\0')
        return false;


    /*
     * Reject values too far outside the expected range.
     *
     * Same intent as the Python:
     *
     * max(abs(...)) > 1.2
     */
    if (fabsf(fl) > 1.2f ||
        fabsf(fr) > 1.2f ||
        fabsf(bl) > 1.2f ||
        fabsf(br) > 1.2f) {

        return false;
    }


    set_pulse(
        FL_PWM_PIN,
        (uint16_t)float_to_pulse(fl)
    );

    set_pulse(
        FR_PWM_PIN,
        (uint16_t)float_to_pulse(fr)
    );

    set_pulse(
        BL_PWM_PIN,
        (uint16_t)float_to_pulse(bl)
    );

    set_pulse(
        BR_PWM_PIN,
        (uint16_t)float_to_pulse(br)
    );

    return true;
}


static bool is_two_digit_number(const char *line)
{
    return (
        line[0] != '\0' &&
        line[1] != '\0' &&
        line[2] == '\0' &&
        isdigit((unsigned char)line[0]) &&
        isdigit((unsigned char)line[1])
    );
}


static uint32_t motor_last_valid_ms;
static bool failsafe_on = false;


static void apply_line(char *line, uint32_t now)
{
    /*
     * Trim leading whitespace.
     */
    char *p = skip_whitespace(line);

    /*
     * Trim trailing whitespace.
     */
    size_t len = strlen(p);

    while (len > 0 &&
           isspace((unsigned char)p[len - 1])) {

        p[len - 1] = '\0';
        len--;
    }


    /*
     * LED command:
     *
     * NN -> NN * 10 ms LED on time
     */
    if (is_two_digit_number(p)) {

        led_on_ms =
            (uint32_t)((p[0] - '0') * 10 +
                       (p[1] - '0')) * 10;

        return;
    }


    /*
     * Motor command:
     *
     * fl,fr,bl,br
     */
    if (apply_wheel_line(p)) {

        motor_last_valid_ms = now;
        failsafe_on = false;
    }
}


// ============================================================
// Encoder reporting
// ============================================================

static uint32_t last_report_ms;


static void print_report(void)
{
    /*
     * Counts are volatile because they are modified by the IRQ.
     *
     * A 32-bit read on RP2040 is atomic, so these individual
     * reads are safe.
     */

    int32_t fl = enc_fl.count;
    int32_t fr = enc_fr.count;
    int32_t bl = enc_bl.count;
    int32_t br = enc_br.count;

    printf("FL %ld\n", (long)fl);
    printf("FR %ld\n", (long)fr);
    printf("BL %ld\n", (long)bl);
    printf("BR %ld\n", (long)br);
}


static void maybe_report(uint32_t now)
{
    if ((uint32_t)(now - last_report_ms) >= REPORT_MS) {

        print_report();

        last_report_ms = now;
    }
}


// ============================================================
// Main
// ============================================================

int main(void)
{
    stdio_init_all();

    // Give USB CDC a moment to initialize.
    sleep_ms(100);


    // --------------------------------------------------------
    // LED
    // --------------------------------------------------------

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    gpio_put(LED_PIN, true);

    led_on_phase = true;
    led_phase_start_ms = to_ms_since_boot(get_absolute_time());


    // --------------------------------------------------------
    // Motors
    // --------------------------------------------------------

    motor_pwm_init();

    neutral_all();


    // --------------------------------------------------------
    // Encoders
    // --------------------------------------------------------

    encoder_init(&enc_fl);
    encoder_init(&enc_fr);
    encoder_init(&enc_bl);
    encoder_init(&enc_br);

    encoder_irq_init();


    // --------------------------------------------------------
    // Timers
    // --------------------------------------------------------

    uint32_t now = to_ms_since_boot(get_absolute_time());

    motor_last_valid_ms = now;
    last_report_ms = now;


    // --------------------------------------------------------
    // Main loop
    // --------------------------------------------------------

    while (true) {

        now = to_ms_since_boot(get_absolute_time());


        // ---------------------------------------------
        // Serial
        // ---------------------------------------------

        read_serial_newest_only();

        if (newest_line_available) {

            apply_line(newest_line, now);

            newest_line_available = false;
        }


        // ---------------------------------------------
        // Motor failsafe
        // ---------------------------------------------

        if ((uint32_t)(now - motor_last_valid_ms) > FAILSAFE_MS) {

            if (!failsafe_on) {

                neutral_all();

                failsafe_on = true;
            }
        }


        // ---------------------------------------------
        // Encoder reporting
        // ---------------------------------------------

        maybe_report(now);


        // ---------------------------------------------
        // LED
        // ---------------------------------------------

        led_update(now);


        /*
         * Match the approximate cadence of the MicroPython
         * implementation.
         *
         * This is NOT required for encoder counting; the
         * encoders are interrupt-driven.
         */
        sleep_ms(1);
    }

    return 0;
}
