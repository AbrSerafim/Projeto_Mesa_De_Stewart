/* Host harness. Include the firmware implementation with GPIO/RTOS stand-ins. */
#define _POSIX_C_SOURCE 200809L
#include <assert.h>
#include <inttypes.h>
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef unsigned uint;
typedef int BaseType_t;
typedef uint32_t TickType_t;
typedef void *SemaphoreHandle_t;
typedef struct { int unused; } pwm_config;
enum { GPIO_OVERRIDE_LOW, GPIO_OVERRIDE_HIGH, GPIO_OVERRIDE_NORMAL, GPIO_FUNC_PWM, GPIO_OUT, clk_sys };
#define portMAX_DELAY UINT32_MAX
#define pdMS_TO_TICKS(ms) (ms)
#define count_of(items) (sizeof(items) / sizeof((items)[0]))

static uint64_t mock_time = 1000000;
static uint32_t inputs = UINT32_MAX, driven;
static unsigned overrides[32], levels[32];
static char response[4096];
static unsigned watchdog_updates;
static int mutex_depth;

static uint64_t time_us_64(void) { return ++mock_time; }
static uint32_t gpio_get_all(void) { return inputs; }
static bool gpio_get(uint pin) { return (driven >> pin) & 1u; }
static void gpio_init(uint pin) { (void)pin; }
static void gpio_put(uint pin, bool level) { driven = (driven & ~(1u << pin)) | ((uint32_t)level << pin); }
static void gpio_put_masked(uint32_t mask, uint32_t value) { driven = (driven & ~mask) | (value & mask); }
static void gpio_pull_up(uint pin) { (void)pin; }
static void gpio_set_dir(uint pin, uint dir) { (void)pin; (void)dir; }
static void gpio_set_function(uint pin, uint function) { (void)pin; (void)function; }
static void gpio_set_outover(uint pin, uint value) { overrides[pin] = value; }
static void pwm_set_gpio_level(uint pin, uint16_t level) { levels[pin] = level; }
static uint pwm_gpio_to_slice_num(uint pin) { return pin / 2; }
static pwm_config pwm_get_default_config(void) { return (pwm_config){0}; }
static void pwm_config_set_wrap(pwm_config *config, uint wrap) { (void)config; (void)wrap; }
static void pwm_config_set_clkdiv(pwm_config *config, float div) { (void)config; (void)div; }
static void pwm_init(uint slice, const pwm_config *config, bool enable) { (void)slice; (void)config; (void)enable; }
static uint clock_get_hz(uint id) { (void)id; return 125000000; }
static void watchdog_update(void) { ++watchdog_updates; }
static void panic(const char *message) { fprintf(stderr, "%s\n", message); abort(); }
static SemaphoreHandle_t xSemaphoreCreateMutex(void) { return &mutex_depth; }
static void xSemaphoreTake(SemaphoreHandle_t lock, unsigned time) { (void)lock; (void)time; assert(mutex_depth++ == 0); }
static void xSemaphoreGive(SemaphoreHandle_t lock) { (void)lock; assert(--mutex_depth == 0); }
static TickType_t xTaskGetTickCount(void) { return (TickType_t)(mock_time / 1000); }
static BaseType_t xTaskDelayUntil(TickType_t *wake, TickType_t delay) { *wake += delay; return 1; }
static unsigned uxTaskGetStackHighWaterMark(void *task) { (void)task; return 100; }

static int capture_printf(const char *format, ...)
{
    assert(mutex_depth == 0);
    const size_t at = strlen(response);
    va_list args;
    va_start(args, format);
    int count = vsnprintf(response + at, sizeof response - at, format, args);
    va_end(args);
    return count;
}

static int capture_puts(const char *text) { return capture_printf("%s\n", text); }
#define printf capture_printf
#define puts capture_puts
#include "../motor_control.c"
#include "../serial_framing.h"
#undef printf
#undef puts

bool imu_capture_reference(uint64_t now) { (void)now; return false; }
bool imu_set_reference(float x, float y, float z) { (void)x; (void)y; (void)z; return false; }
void imu_zero_heading(void) {}
void imu_send_reference(void) {}

static void near(float left, float right) { assert(fabsf(left - right) < .002f); }

static void command(const char *input, const char *expected)
{
    char line[192];
    snprintf(line, sizeof line, "%s", input);
    response[0] = '\0';
    motor_command(line);
    assert(strstr(response, expected) != NULL);
}

static void test_framing(void)
{
    SerialFrame frame = {0};
    const char *text = "BALL2 1 2 40 41\r\nSTOP\n";
    for (unsigned i = 0; text[i]; ++i) {
        if (serial_frame_feed(&frame, text[i], i * 1000) == RX_COMMAND) {
            assert(strcmp(frame.line, i == 16 ? "BALL2 1 2 40 41" : "STOP") == 0);
        }
    }
    for (unsigned i = 0; i < 194; ++i) {
        assert(serial_frame_feed(&frame, 'x', 50000) == RX_NONE);
    }
    assert(serial_frame_feed(&frame, '\n', 50001) == RX_INVALID);
    assert(serial_frame_feed(&frame, 'B', 51000) == RX_NONE);
    assert(serial_frame_feed(&frame, '\n', 51000 + RX_FRAGMENT_TIMEOUT_US + 1) == RX_INVALID);
    assert(serial_frame_feed(&frame, 'A', 600000) == RX_NONE);
    assert(serial_frame_feed(&frame, 0x18, 600001) == RX_NONE);
    for (const char *stop = "STOP"; *stop; ++stop) {
        assert(serial_frame_feed(&frame, *stop, 600002) == RX_NONE);
    }
    assert(serial_frame_feed(&frame, '\n', 600003) == RX_COMMAND);
    assert(strcmp(frame.line, "STOP") == 0);
}

static void test_protocol_and_safety(void)
{
    command("HELLO", "!HELLO,13");
    command("BALLLINK2 2468", "!BALLLINK2,2468");
    command("BALLTEST2 4294967297 2468", "!BALLSTARTERR,0,argumento_invalido");
    command("BALLTEST2 4000000001 2468", "!BALLSTATE2,4000000001,5");
    for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
        assert(overrides[pins[i].enable] == GPIO_OVERRIDE_LOW);
    }
    command("GO 1 1 40", "!ERR,GO,bola_em_andamento");
    command("BALL2 4000000001 4000000002 39 43", "!BALLACK,4000000001,4000000002,5,39.0000,43.0000");
    command("BALL2 4000000001 4000000002 39 43", "!BALLNACK,4000000001,4000000002,sequencia_antiga");
    command("BALL2 4000000001 4000000003 39 NaN", "!BALLNACK,4000000001,4000000003,argumento_invalido");
    command("BALL2 4000000001 4000000003 39 Inf", "!BALLNACK,4000000001,4000000003,argumento_invalido");
    command("BALL2 4000000001 4000000003 39 43 45", "!BALLNACK,4000000001,4000000003,argumento_invalido");
    command("BALL2 4000000001 4000000003 39 100", "!BALLNACK,4000000001,4000000003,bola_fora_da_faixa");
    command("STOP", "!OK,STOP");
    command("BALL2 4000000001 4000000004 39 43", "!BALLNACK,4000000001,4000000004,sessao_inativa");
    command("BALLTEST2 4000000001 2468", "!BALLSTARTERR,4000000001,sessao_ocupada");

    command("HELLO", "!HELLO,13");
    command("BALLLINK2 1357", "!BALLLINK2,1357");
    command("BALLTEST2 4000000002 2468", "!BALLSTARTERR,4000000002,sem_sessao");
    for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
        Motor *m = &motors[i];
        m->span = 100;
        m->travel = (TravelCalibration){.min_count = 0, .max_count = 356, .captured = LIMIT_BOTH};
        m->position = 40;
        m->sampled_at = mock_time;
    }
    command("BALLSTART2 3 1357 10 90 45 40 12 88 45 42", "!BALLSTATE2,3,4");
    near(motors[MOTOR_1].goal, 40);
    near(motors[MOTOR_3].goal, 42);
    command("BALL2 3 1 38 46", "!BALLACK,3,1,4,38.0000,46.0000");
    near(motors[MOTOR_1].goal, 40);
    near(motors[MOTOR_3].goal, 42);
    for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
        motors[i].capture_at = mock_time;
        motors[i].position = motors[i].goal;
    }
    ball_center_update(mock_time);
    mock_time += HOME_SETTLE_US;
    ball_center_update(mock_time);
    assert(ball_state == BALL_ACTIVE);
    command("BALL2 3 2 38 46", "!BALLACK,3,2,1,38.0000,46.0000");
    near(motors[MOTOR_1].goal, 38);
    near(motors[MOTOR_3].goal, 46);
    assert(!capturing_position(&motors[MOTOR_1], mock_time));
    command("BALL2 3 3 87 97", "!BALLNACK,3,3,bola_fora_da_faixa");
    assert(ball_state == BALL_MOTOR && motors[MOTOR_3].fault == BALL_INVALID_TARGET);
    assert(motors[MOTOR_1].mode == BRAKE && motors[MOTOR_3].mode == BRAKE);

    for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
        motors[i].sampled_at = mock_time;
    }
    command("BALLSTART2 4 1357 10 90 45 40 12 88 45 42", "!BALLSTARTERR,4,rearme_necessario");
    command("REARM", "!OK,REARM");
    command("BALLSTART2 4 1357 10 90 45 40 12 88 45 42", "!BALLSTATE2,4,4");
    mock_time += BALL_TIMEOUT_US;
    update_motor(MOTOR_1, mock_time, true, true);
    assert(ball_state == BALL_TIMEOUT && motors[MOTOR_1].mode == BRAKE);
    motor_link_down();
    assert(!linked && motors[MOTOR_3].mode == BRAKE);
    command("HELLO", "!HELLO,13");
    command("BALL2 4 1 38 46", "!BALLNACK,4,1,sessao_inativa");
}

static void test_bridge_and_friction(void)
{
    const unsigned i = MOTOR_1;
    const Pins *p = &pins[i];
    Motor *m = &motors[i];
    assert(ENCODER_DEGREES_PER_COUNT == .28125f);
    m->mode = POSITION;
    m->position = 40;
    m->cfg.duty = 60;
    neutral_output(i, false);
    assert(!gpio_get(p->in_positive) && !gpio_get(p->in_negative));
    assert(overrides[p->enable] == GPIO_OVERRIDE_LOW);
    bridge_output(i, 20, false, mock_time);
    bridge_output(i, 20, false, mock_time += REVERSAL_US);
    assert(gpio_get(p->in_positive) && !gpio_get(p->in_negative));
    near(m->output, 20);
    bridge_output(i, -20, false, ++mock_time);
    assert(gpio_get(p->in_positive) && gpio_get(p->in_negative));
    assert(overrides[p->enable] == GPIO_OVERRIDE_HIGH);
    bridge_output(i, -20, false, mock_time += SAMPLE_US);
    bridge_output(i, -20, false, mock_time += REVERSAL_US - 1);
    assert(m->output == 0 && !levels[p->enable]);
    bridge_output(i, -20, false, ++mock_time);
    assert(!gpio_get(p->in_positive) && gpio_get(p->in_negative));
    near(m->output, -20);
    inputs &= ~(1u << p->min_switch);
    bridge_output(i, -20, false, ++mock_time);
    assert(m->output == 0 && m->bridge == BRIDGE_BRAKE);
    inputs = UINT32_MAX;
    command("FRICTION 1 17.2 23 11 15", "!OK,FRICTION");
    m->encoder_motion_at = mock_time;
    reset_speed_pi(m);
    near(speed_pi(m, mock_time, true, 2, 2), 11.07f);
    near(speed_pi(m, mock_time, true, -2, -2), -15.07f);
    m->running_forward = 0;
    near(speed_pi(m, mock_time, true, 2, 2), 13.07f);
}

static void test_encoder_and_watchdog(void)
{
    mock_time += 5000;
    for (unsigned j = 0; j <= 30; ++j) {
        const int64_t base = INT64_C(4294967290);
        motor_encoder_sample(MOTOR_1, base + j, mock_time += SAMPLE_US);
        motor_encoder_sample(MOTOR_3, base - j, mock_time);
    }
    near(motors[MOTOR_1].control_velocity, 56.25f);
    near(motors[MOTOR_3].velocity, -56.25f);
    motors[MOTOR_3].report_at = mock_time;
    motors[MOTOR_1].mode = motors[MOTOR_3].mode = BRAKE;
    unsigned prior = watchdog_updates;
    update_motor(MOTOR_1, mock_time + 1, true, true);
    assert(watchdog_updates == prior + 1);
    motors[MOTOR_3].mode = POSITION;
    update_motor(MOTOR_3, mock_time + 4 * SAMPLE_US, true, true);
    assert(motors[MOTOR_3].fault == ENCODER_STALE && motors[MOTOR_3].mode == BRAKE);
}

int main(void)
{
    motors_init();
    test_framing();
    test_protocol_and_safety();
    test_bridge_and_friction();
    test_encoder_and_watchdog();
    puts("Firmware host checks passed (framing, BALL, tracking, guards, bridge and friction).");
    return 0;
}
