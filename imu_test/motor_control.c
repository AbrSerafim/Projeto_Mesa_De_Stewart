#include "motor_control.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "hardware/watchdog.h"
#include "pico/stdlib.h"

#define CONTROL_MS 20u
#define CONTROL_DT (CONTROL_MS / 1000.0f)
#define SAMPLE_US (ENCODER_PERIOD_MS * 1000u)
#define HISTORY_SAMPLES 30u
#define LINK_TIMEOUT_US 1000000u
#define PHASE_TIMEOUT_US 45000000u
#define START_ASSIST_DELAY_US 150000u
#define START_ASSIST_RATE 55.0f
#define MEASURED_START_ASSIST_RATE 85.0f
#define STALL_AT_LIMIT_US 750000u
#define STEP_DURATION_US 6000000u
#define PWM_TOP 6249u
#define ENABLE_PWM_HZ 20000u
#define PWM_RISE_PER_TICK (25.0f * ENCODER_PERIOD_MS / 1000.0f)
#define REVERSAL_US 30000u
#define CAL_PAUSE_US 80000u
#define SPEED_GUARD_WINDOW_US 150000u
#define SPEED_PI_HOLD_US 100000u
#define SPEED_CONTROL_SAMPLES (50u / ENCODER_PERIOD_MS)
// The speed PI takes an error in degrees/second and outputs duty in percent.
#define SPEED_PI_KP 0.20f
#define SPEED_PI_KI 0.30f
// Approximate inverse slope of the M1/M3 bench data, in % PWM per degree/second.
#define SPEED_FEEDFORWARD 0.035f
#define FRICTION_DUTY_PERCENT 13.0f
#define FRICTION_BLEND_SPEED 1.0f
#define POSITION_CAPTURE_DEG ENCODER_DEGREES_PER_COUNT
#define POSITION_RELEASE_DEG (2 * ENCODER_DEGREES_PER_COUNT)
#define POSITION_BRAKE_US 150000u
#define POSITION_SPEED_DELAY_S 0.10f
#define LEVEL_MARGIN_DEG 3.0f
#define LEVEL_PROBE_DEG 4.0f
#define LEVEL_TOLERANCE_DEG .5f
#define LEVEL_STAGE_US 12000000u
#define LEVEL_TOTAL_US 75000000u
#define LEVEL_IMU_MAX_AGE_US 100000u
#define LEVEL_SAMPLES 16u
#define LEVEL_SETTLE_US 300000u
#define RAD_TO_DEG 57.29577951f
#define HOME_TRAVEL_FRACTION .40f
#define HOME_SETTLE_US 200000u
#define HOME_TIMEOUT_US 45000000u
#define HOME_PERCENT (100.0f * HOME_TRAVEL_FRACTION)
#define BALL_TRAVEL_MIN 1.0f
#define BALL_TRAVEL_MAX 99.0f
#define BALL_TIMEOUT_US 450000u
#define BALL_SWITCH_CONFIRM_US 25000u
#define BALL_START_ASSIST_DELAY_US 60000u
#define BALL_START_ASSIST_RATE 110.0f
#define BALL_MEASURED_START_ASSIST_RATE 160.0f
#define PWM_TEST_TIMEOUT_US 350000u
#define PWM_TEST_MARGIN_DEG 2.0f
#define PWM_TEST_RISE_PER_TICK (5.0f * ENCODER_PERIOD_MS / 1000.0f)

typedef struct {
    uint in_positive, in_negative, enable, min_switch, max_switch;
} Pins;

static const Pins pins[MOTOR_COUNT] = {
    {2, 3, 0, 10, 11},   // M1: IN1/IN2, ENA
    {15, 14, 1, 12, 13}, // M3: IN4/IN3, ENB
};

typedef enum { CONTROL_P, CONTROL_PI, CONTROL_PID, CONTROL_PIDF } ControlLaw;

typedef struct {
    float kp, ki, kd, tf;
    float speed, acceleration, cal_fast, cal_slow, duty;
    int polarity;
    ControlLaw law;
} Settings;

typedef enum { BRAKE, COAST, CALIBRATE, POSITION, STEP, SINE, COSINE, RAMP, PWM_TEST } Mode;
typedef enum { SEEK_FAST, BACK_OFF, SEEK_SLOW, CLEAR_SWITCH } Phase;
typedef enum {
    NO_FAULT, LINK_LOST, SWITCH_CONFLICT, WRONG_SWITCH, CAL_TIMEOUT,
    ENCODER_STALE, STALLED, INVALID_SPAN, UNEXPECTED_SWITCH, OVERSPEED, OUT_OF_TRAVEL,
    BALL_ENDSTOP, BALL_OUTSIDE_WINDOW, BALL_CENTER_TIMEOUT, BALL_INVALID_TARGET,
} Fault;

typedef enum {
    LEVEL_IDLE, LEVEL_BASELINE, LEVEL_PROBE_M1, LEVEL_PROBE_M3, LEVEL_CORRECT,
    LEVEL_DONE, LEVEL_INTERRUPTED, LEVEL_IMU, LEVEL_TIMEOUT, LEVEL_GEOMETRY,
    LEVEL_TRAVEL, LEVEL_DIVERGED, LEVEL_MOTOR,
} LevelState;

typedef enum { HOME_IDLE, HOME_MOVING, HOME_BRAKED, HOME_INTERRUPTED, HOME_TIMEOUT, HOME_MOTOR } HomeState;
typedef enum { BALL_IDLE, BALL_ACTIVE, BALL_TIMEOUT, BALL_MOTOR, BALL_CENTERING } BallState;

typedef struct {
    float x, y;
} Tilt;

typedef struct {
    LevelState state;
    uint64_t started_at, stage_at, quiet_at, consumed_at;
    unsigned samples, corrections, no_progress;
    float previous_error, tilt;
    Tilt sum, first, probe_tilt[2], axes[2];
    float probe_pos[2][MOTOR_COUNT], target[MOTOR_COUNT], result[MOTOR_COUNT];
} Leveling;

typedef struct {
    Settings cfg;
    TravelCalibration travel;
    Mode mode;
    Phase phase;
    Fault fault;
    unsigned endpoint, raw, stable, previous_raw;
    int last_direction;
    int64_t count, last_count;
    int64_t counts[HISTORY_SAMPLES];
    uint64_t times[HISTORY_SAMPLES];
    unsigned speed_slot;
    uint64_t sampled_at, phase_at, stalled_at, started_at, report_at, drive_at;
    uint64_t guard_at, speed_pi_at;
    uint64_t capture_at;
    uint64_t ball_switch_at, encoder_motion_at;
    int64_t guard_count;
    bool guard_excess;
    uint32_t step_id;
    float step_elapsed;
    float velocity, position, goal, reference, ref_velocity, output, requested;
    float fault_speed, fault_threshold;
    float previous_goal_error, control_velocity;
    float breakaway_forward, breakaway_reverse;
    float velocity_target, speed_setpoint, speed_integral;
    int speed_direction;
    float integral, derivative, previous_angle, ki_ts, alpha, beta;
    float baseline, amplitude, period;
    float span, ball_min, ball_max, ball_duty;
} Motor;

static Motor motors[MOTOR_COUNT] = {
    {.cfg = {400, 20, 10, .105f, 30, 90, 8, 4, 30, 1, CONTROL_PIDF}},
    {.cfg = {400, 20, 10, .105f, 30, 90, 8, 4, 30, 1, CONTROL_PIDF}},
};

static SemaphoreHandle_t lock;
static uint64_t last_heartbeat;
static bool linked;
static bool motion_limits_enabled = true;
static uint32_t next_step_id;
static Leveling leveling;
static HomeState home_state;
static uint64_t home_started_at, home_quiet_at;
static BallState ball_state;
static uint64_t ball_command_at, ball_center_started_at, ball_center_quiet_at;
typedef struct {
    unsigned motor;
    Fault fault;
    float position, target, duty, speed, threshold;
} BallFault;
static BallFault ball_fault;
typedef enum { PWM_IDLE, PWM_RUNNING, PWM_FOUND, PWM_INTERRUPTED } PwmTestState;
static struct {
    PwmTestState state;
    unsigned motor;
    int direction;
    int64_t start_count;
    uint64_t received_at;
    float target, first_duty;
} pwm_test;
static struct {
    float x, y, z, gyro;
    uint64_t at;
} level_imu;

static bool level_active(void)
{
    return leveling.state >= LEVEL_BASELINE && leveling.state <= LEVEL_CORRECT;
}

static bool ball_running(void)
{
    return ball_state == BALL_ACTIVE || ball_state == BALL_CENTERING;
}

static bool limits_for_motion(void)
{
    return motion_limits_enabled || home_state == HOME_MOVING || ball_running();
}

static float speed_limit(const Motor *m)
{
    return level_active() || ball_running() ? fminf(m->cfg.speed, 20) : m->cfg.speed;
}

static float acceleration_limit(const Motor *m)
{
    return level_active() || ball_running() ? fminf(m->cfg.acceleration, 60) : m->cfg.acceleration;
}

static float duty_limit(const Motor *m)
{
    const float limit = ball_running() ? m->ball_duty : level_active() ? 30 : 100;
    return fminf(m->cfg.duty, limit);
}

static float clamp(float value, float low, float high)
{
    return fminf(high, fmaxf(low, value));
}

static float approach(float value, float target, float delta)
{
    return value + clamp(target - value, -delta, delta);
}

static unsigned read_switches(const Pins *p)
{
    const uint32_t levels = gpio_get_all();
    return ((levels & (1u << p->min_switch)) == 0 ? LIMIT_MIN : 0)
         | ((levels & (1u << p->max_switch)) == 0 ? LIMIT_MAX : 0);
}

static void set_direction(const Pins *p, int direction)
{
    gpio_put_masked((1u << p->in_positive) | (1u << p->in_negative),
                    direction > 0 ? 1u << p->in_positive : direction < 0 ? 1u << p->in_negative : 0);
}

static void idle_bridge(const Pins *p, bool brake)
{
    gpio_set_outover(p->enable, GPIO_OVERRIDE_LOW);
    pwm_set_gpio_level(p->enable, 0);
    const uint32_t inputs = (1u << p->in_positive) | (1u << p->in_negative);
    gpio_put_masked(inputs, brake && home_state == HOME_BRAKED ? inputs : 0);
    if (brake) {
        gpio_set_outover(p->enable, GPIO_OVERRIDE_HIGH);
    }
}

static void neutral_output(unsigned i, bool brake)
{
    Motor *m = &motors[i];
    idle_bridge(&pins[i], brake);
    m->output = 0;
    m->last_direction = 0;
}

static void bridge_output(unsigned i, float duty, bool brake_at_zero, uint64_t now)
{
    Motor *m = &motors[i];
    const Pins *p = &pins[i];
    const unsigned raw = read_switches(p);
    const bool toward_min = duty < 0 || m->output < 0;
    const bool toward_max = duty > 0 || m->output > 0;
    if (m->mode == BRAKE || raw == LIMIT_BOTH
        || (toward_min && ((raw & LIMIT_MIN) || (m->mode >= POSITION && m->position <= 0)))
        || (toward_max && ((raw & LIMIT_MAX) || (m->mode >= POSITION && m->position >= m->span)))
        || (m->mode == CALIBRATE && (m->phase == SEEK_FAST || m->phase == SEEK_SLOW)
            && (raw & m->endpoint))) {
        neutral_output(i, true);
        return;
    }
    if (m->mode == COAST || (m->mode == CALIBRATE && now - m->phase_at < CAL_PAUSE_US)) {
        neutral_output(i, m->mode == CALIBRATE);
        return;
    }
    const int direction = duty == 0 ? 0 : (duty > 0 ? 1 : -1) * m->cfg.polarity;
    if (m->output == 0 && direction != 0 && direction != m->last_direction) {
        idle_bridge(p, true);
        m->drive_at = now + (m->last_direction == 0 ? SAMPLE_US : REVERSAL_US);
        m->last_direction = direction;
        return;
    }
    if (m->output * duty < 0 || (m->output == 0 && now < m->drive_at)) {
        idle_bridge(p, true);
        m->output = 0;
        return;
    }
    const float max_duty = duty_limit(m);
    const float target = clamp(duty, -max_duty, max_duty);
    const float rise = m->mode == PWM_TEST ? PWM_TEST_RISE_PER_TICK
        : m->mode == CALIBRATE ? PWM_RISE_PER_TICK : INFINITY;
    const float next = fabsf(target) > fabsf(m->output) ? approach(m->output, target, rise) : target;
    const uint16_t level = (uint16_t)roundf(fabsf(next) * (PWM_TOP + 1) / 100.0f);
    if (level == 0) {
        // A zero effort during speed regulation disables EN. Explicit stops and limits brake.
        idle_bridge(p, brake_at_zero || raw != 0);
        m->output = 0;
        return;
    }
    if (m->output == 0) {
        gpio_set_outover(p->enable, GPIO_OVERRIDE_LOW);
        set_direction(p, m->last_direction);
    }
    pwm_set_gpio_level(p->enable, level);
    gpio_set_outover(p->enable, GPIO_OVERRIDE_NORMAL);
    m->output = next;
}

static void update_coefficients(Motor *m)
{
    const float tf = m->cfg.law == CONTROL_PIDF ? m->cfg.tf : 0;
    m->ki_ts = m->cfg.law >= CONTROL_PI ? m->cfg.ki * CONTROL_DT : 0;
    m->alpha = tf / (tf + CONTROL_DT);
    m->beta = m->cfg.law >= CONTROL_PID ? m->cfg.kd / (tf + CONTROL_DT) : 0;
}

static float position_integral_limit(const Motor *m)
{
    return .25f * (limits_for_motion() ? speed_limit(m) : 360.0f);
}

static void reset_speed_pi(Motor *m)
{
    m->speed_pi_at = 0;
    m->speed_setpoint = 0;
    m->speed_integral = 0;
    m->speed_direction = 0;
    m->requested = 0;
}

static void reset_control(Motor *m)
{
    m->reference = m->position;
    m->previous_angle = m->position;
    m->ref_velocity = 0;
    m->integral = 0;
    m->derivative = 0;
    m->velocity_target = 0;
    reset_speed_pi(m);
    m->guard_at = 0;
    m->stalled_at = 0;
    m->guard_excess = false;
    m->capture_at = 0;
    m->ball_switch_at = 0;
}

static void invalidate_travel(Motor *m)
{
    m->travel = (TravelCalibration){0};
    m->span = 0;
}

static void stop_all(Mode mode, Fault fault)
{
    if (pwm_test.state == PWM_RUNNING) {
        pwm_test.state = PWM_INTERRUPTED;
    }
    if (ball_running()) {
        ball_state = BALL_IDLE;
    }
    if (home_state == HOME_MOVING) {
        home_state = HOME_INTERRUPTED;
    } else if (home_state == HOME_BRAKED) {
        home_state = HOME_IDLE;
    }
    if (level_active()) {
        leveling.state = LEVEL_INTERRUPTED;
    }
    for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
        Motor *m = &motors[i];
        if (m->mode == CALIBRATE) {
            invalidate_travel(m);
        }
        m->mode = mode;
        if (fault != NO_FAULT && (fault != LINK_LOST || m->fault == NO_FAULT)) {
            m->fault = fault;
        }
        reset_control(m);
        neutral_output(i, mode == BRAKE);
    }
}

static void trip_motor(unsigned i, Fault fault)
{
    const bool homing = home_state == HOME_MOVING;
    const bool balancing = ball_running();
    if (balancing) {
        const Motor *m = &motors[i];
        ball_fault.motor = i == MOTOR_1 ? 1 : 3;
        ball_fault.fault = fault;
        ball_fault.position = m->span > 0 ? 100 * m->position / m->span : 0;
        ball_fault.target = m->span > 0 ? 100 * m->goal / m->span : 0;
        ball_fault.duty = m->output;
        ball_fault.speed = fault == OVERSPEED ? m->fault_speed : fabsf(m->velocity);
        ball_fault.threshold = fault == OVERSPEED ? m->fault_threshold : 0;
    }
    if (level_active()) {
        leveling.state = LEVEL_MOTOR;
    }
    motors[i].fault = fault;
    stop_all(BRAKE, NO_FAULT);
    if (homing) {
        home_state = HOME_MOTOR;
    }
    if (balancing) {
        ball_state = BALL_MOTOR;
    }
}

void motor_imu_sample(float ax, float ay, float az, float gyro_norm, uint64_t sampled_at)
{
    xSemaphoreTake(lock, portMAX_DELAY);
    level_imu.x = ax;
    level_imu.y = ay;
    level_imu.z = az;
    level_imu.gyro = gyro_norm;
    level_imu.at = sampled_at;
    xSemaphoreGive(lock);
}

void motors_init(void)
{
    lock = xSemaphoreCreateMutex();
    if (lock == NULL) {
        panic("Could not allocate motor mutex");
    }
    for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
        const Pins *p = &pins[i];
        const uint outputs[] = {p->enable, p->in_positive, p->in_negative};
        for (unsigned n = 0; n < count_of(outputs); ++n) {
            gpio_init(outputs[n]);
            gpio_put(outputs[n], 0);
            gpio_set_dir(outputs[n], GPIO_OUT);
        }
        const uint switches[] = {p->min_switch, p->max_switch};
        for (unsigned n = 0; n < count_of(switches); ++n) {
            gpio_init(switches[n]);
            gpio_pull_up(switches[n]);
        }
        gpio_set_function(p->enable, GPIO_FUNC_PWM);
        gpio_set_outover(p->enable, GPIO_OVERRIDE_LOW);
        update_coefficients(&motors[i]);
    }
    // GPIO0 and GPIO1 share one PWM slice, with independent channel compare values.
    const uint slice = pwm_gpio_to_slice_num(pins[MOTOR_1].enable);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_wrap(&config, PWM_TOP);
    pwm_config_set_clkdiv(&config, (float)clock_get_hz(clk_sys) / ((PWM_TOP + 1u) * ENABLE_PWM_HZ));
    pwm_init(slice, &config, true);
}

void motor_encoder_sample(unsigned i, int64_t count, uint64_t sampled_at)
{
    xSemaphoreTake(lock, portMAX_DELAY);
    Motor *m = &motors[i];
    const unsigned oldest = m->speed_slot;
    if (m->times[oldest] != 0 && sampled_at > m->times[oldest]) {
        const float dt = (sampled_at - m->times[oldest]) * 1e-6f;
        m->velocity = (float)(count - m->counts[oldest]) * ENCODER_DEGREES_PER_COUNT / dt;
    }
    const unsigned recent = (oldest + HISTORY_SAMPLES - SPEED_CONTROL_SAMPLES) % HISTORY_SAMPLES;
    if (m->times[recent] != 0 && sampled_at > m->times[recent]) {
        const float dt = (sampled_at - m->times[recent]) * 1e-6f;
        m->control_velocity = (float)(count - m->counts[recent]) * ENCODER_DEGREES_PER_COUNT / dt;
    }
    m->counts[oldest] = count;
    m->times[oldest] = sampled_at;
    m->speed_slot = (oldest + 1) % HISTORY_SAMPLES;
    m->count = count;
    m->sampled_at = sampled_at;
    xSemaphoreGive(lock);
}

static void enter_phase(Motor *m, Phase phase, uint64_t now)
{
    m->phase = phase;
    m->phase_at = now;
    m->encoder_motion_at = now;
    reset_control(m);
}

static void begin_calibration(Motor *m, uint64_t now)
{
    invalidate_travel(m);
    m->mode = CALIBRATE;
    m->fault = NO_FAULT;
    m->endpoint = LIMIT_MIN;
    enter_phase(m, SEEK_FAST, now);
}

static int encoder_direction(const Motor *m)
{
    return m->travel.max_count >= m->travel.min_count ? 1 : -1;
}

static bool overspeed(Motor *m, float max_speed)
{
    if (!limits_for_motion()) {
        return false;
    }
    // Independent windows prevent one encoder burst from counting twice.
    // Arm after the bridge starts driving this phase, excluding the preceding reversal.
    if (m->guard_at == 0) {
        if (m->output != 0) {
            m->guard_at = m->sampled_at;
            m->guard_count = m->count;
        }
        return false;
    }
    const uint64_t elapsed = m->sampled_at - m->guard_at;
    if (elapsed < SPEED_GUARD_WINDOW_US) {
        return false;
    }
    const float dt = elapsed * 1e-6f;
    const float speed = fabsf((float)(m->count - m->guard_count)) * ENCODER_DEGREES_PER_COUNT / dt;
    const float threshold = max_speed + 2 * ENCODER_DEGREES_PER_COUNT / dt;
    const bool excessive = speed > threshold;
    const bool trip = excessive && m->guard_excess;
    m->guard_at = m->sampled_at;
    m->guard_count = m->count;
    m->guard_excess = excessive;
    if (trip) {
        m->fault_speed = speed;
        m->fault_threshold = threshold;
    }
    return trip;
}

static float startup_step(const Motor *m, uint64_t now, int direction, float effort, float dt)
{
    const bool balancing = ball_running();
    const uint64_t delay = balancing ? BALL_START_ASSIST_DELAY_US : START_ASSIST_DELAY_US;
    if (now - m->encoder_motion_at < delay) {
        return 0;
    }
    const float measured = direction > 0 ? m->breakaway_forward : m->breakaway_reverse;
    if (balancing) {
        return (effort < measured ? BALL_MEASURED_START_ASSIST_RATE : BALL_START_ASSIST_RATE) * dt;
    }
    const float rate = effort < measured ? MEASURED_START_ASSIST_RATE : START_ASSIST_RATE;
    return rate * dt;
}

static float integrate_duty(float integral, float step, float base, float minimum, float maximum)
{
    const float effort = base + integral;
    // Accept only the part of the increment that can change the applied duty.
    return clamp(integral + clamp(step, fminf(0, -effort), fmaxf(0, maximum - effort)), minimum, maximum);
}

static bool stalled_at_limit(Motor *m, uint64_t now)
{
    const float limit = duty_limit(m);
    if (limit <= 0 || fabsf(m->output) + .03f < limit || now - m->encoder_motion_at < START_ASSIST_DELAY_US) {
        m->stalled_at = 0;
        return false;
    }
    if (m->stalled_at == 0) {
        m->stalled_at = now;
    }
    if (now - m->stalled_at < STALL_AT_LIMIT_US) {
        return false;
    }
    m->fault_speed = m->output;
    m->fault_threshold = limit;
    return true;
}

static float calibration_update(unsigned i, uint64_t now, bool control_sample)
{
    Motor *m = &motors[i];
    if (m->raw & (LIMIT_BOTH ^ m->endpoint)) {
        trip_motor(i, WRONG_SWITCH);
        return 0;
    }
    if (now - m->phase_at > PHASE_TIMEOUT_US) {
        trip_motor(i, CAL_TIMEOUT);
        return 0;
    }
    const bool seeking = m->phase == SEEK_FAST || m->phase == SEEK_SLOW;
    const bool active = (m->stable & m->endpoint) != 0;
    if (seeking && active && (m->raw & m->endpoint)) {
        if (m->phase == SEEK_FAST) {
            enter_phase(m, BACK_OFF, now);
        } else {
            if (m->endpoint == LIMIT_MIN) {
                m->travel.min_count = m->count;
                m->travel.captured = LIMIT_MIN;
            } else {
                if (llabs(m->count - m->travel.min_count) < 16) {
                    trip_motor(i, INVALID_SPAN);
                    return 0;
                }
                m->travel.max_count = m->count;
            }
            enter_phase(m, CLEAR_SWITCH, now);
        }
        return 0;
    }
    if (!seeking && !active && !(m->raw & m->endpoint)) {
        if (m->phase == BACK_OFF) {
            enter_phase(m, SEEK_SLOW, now);
        } else if (m->endpoint == LIMIT_MIN) {
            m->endpoint = LIMIT_MAX;
            enter_phase(m, SEEK_FAST, now);
        } else {
            m->span = fabsf((float)(m->travel.max_count - m->travel.min_count)) * ENCODER_DEGREES_PER_COUNT;
            m->travel.captured = LIMIT_BOTH;
            m->position = (float)(m->count - m->travel.min_count)
                * ENCODER_DEGREES_PER_COUNT * encoder_direction(m);
            m->goal = m->position;
            reset_control(m);
            m->mode = BRAKE;
            if (i == MOTOR_1 && read_switches(&pins[MOTOR_3]) == 0) {
                begin_calibration(&motors[MOTOR_3], now);
            } else if (i == MOTOR_1) {
                trip_motor(MOTOR_3, WRONG_SWITCH);
            }
        }
        return 0;
    }
    if (seeking && (m->raw & m->endpoint)) {
        return 0;
    }
    if (stalled_at_limit(m, now)) {
        trip_motor(i, STALLED);
        return 0;
    }
    const int direction = (m->endpoint == LIMIT_MIN ? -1 : 1) * (seeking ? 1 : -1);
    const float speed = m->phase == SEEK_FAST ? m->cfg.cal_fast : m->cfg.cal_slow;
    // Homing speeds are PI references. The configured maximum protects both modes.
    if (overspeed(m, m->cfg.speed)) {
        trip_motor(i, OVERSPEED);
        return 0;
    }
    if (control_sample && now - m->phase_at >= CAL_PAUSE_US) {
        m->ref_velocity = motion_limits_enabled
            ? approach(m->ref_velocity, speed, m->cfg.acceleration * CONTROL_DT) : speed;
        const float error = m->ref_velocity - fabsf(m->velocity);
        const float proportional = .75f * error;
        const float step = 3.0f * CONTROL_DT * error;
        const float start = error > 0 ? startup_step(m, now, direction, proportional + m->integral, CONTROL_DT) : 0;
        m->integral = integrate_duty(m->integral, step + start, proportional, 0, m->cfg.duty);
        m->requested = direction * clamp(m->integral + proportional, 0, m->cfg.duty);
    }
    return m->requested;
}

static bool capturing_position(Motor *m, uint64_t now)
{
    if (m->mode != POSITION && m->mode != STEP) {
        return false;
    }
    const float error = m->goal - m->position;
    const bool ball_neutral = ball_running() && fabsf(m->goal - HOME_TRAVEL_FRACTION * m->span) < .0001f;
    const float capture_distance = ball_neutral ? POSITION_RELEASE_DEG : POSITION_CAPTURE_DEG;
    const bool crossed = error * m->previous_goal_error < 0;
    m->previous_goal_error = error;
    if (m->capture_at != 0) {
        if (fabsf(error) <= POSITION_RELEASE_DEG || now - m->capture_at < POSITION_BRAKE_US
            || fabsf(m->velocity) >= ENCODER_DEGREES_PER_COUNT / .15f) {
            return true;
        }
        m->capture_at = 0;
        m->previous_angle = m->position;
        return false;
    }
    if (!crossed && fabsf(error) > capture_distance) {
        return false;
    }
    m->capture_at = now;
    m->integral = m->derivative = m->velocity_target = 0;
    reset_speed_pi(m);
    return true;
}

static float speed_bound(float distance, const Motor *m)
{
    const float accel = acceleration_limit(m);
    const float reaction = accel * POSITION_SPEED_DELAY_S;
    return fminf(speed_limit(m), sqrtf(2 * accel * fmaxf(0, distance) + reaction * reaction) - reaction);
}

static float speed_pi(Motor *m, uint64_t now, bool control_sample, float target, float measured)
{
    if (fabsf(target) < .05f) {
        if (m->speed_pi_at != 0 && now - m->speed_pi_at >= SPEED_PI_HOLD_US) {
            reset_speed_pi(m);
        }
        m->speed_setpoint = m->requested = 0;
        return 0;
    }
    const int direction = target > 0 ? 1 : -1;
    if (direction != m->speed_direction) {
        reset_speed_pi(m);
        m->speed_direction = direction;
    }
    m->speed_setpoint = target;
    if (!control_sample && m->speed_pi_at != 0) {
        return m->requested;
    }
    const float dt = m->speed_pi_at == 0 ? 0 : fminf((now - m->speed_pi_at) * 1e-6f, 2 * CONTROL_DT);
    m->speed_pi_at = now;
    const float speed = fabsf(target);
    const float error = speed - direction * measured;
    const float max_duty = duty_limit(m);
    const float friction = fminf(FRICTION_DUTY_PERCENT, max_duty * .60f);
    const float feedforward = fminf(max_duty * .60f,
        friction * fminf(1, speed / FRICTION_BLEND_SPEED) + SPEED_FEEDFORWARD * speed);
    const float base = feedforward + SPEED_PI_KP * error;
    const float step = SPEED_PI_KI * dt * error;
    // Add starting torque to the integral, so it remains available after the first encoder edges.
    const float start = error > 0 ? startup_step(m, now, direction, base + m->speed_integral, dt) : 0;
    m->speed_integral = integrate_duty(m->speed_integral, step + start, base, -friction, max_duty);
    m->requested = direction * clamp(base + m->speed_integral, 0, max_duty);
    return m->requested;
}

static void reference_update(Motor *m, uint64_t now)
{
    const float elapsed = (now - m->started_at) * 1e-6f;
    if (m->mode == SINE || m->mode == COSINE) {
        const float phase = 6.28318530718f * fmodf(elapsed, m->period) / m->period;
        m->goal = m->baseline + m->amplitude * (m->mode == SINE ? sinf(phase) : cosf(phase));
    } else if (m->mode == RAMP) {
        m->goal = m->baseline + m->amplitude * clamp(elapsed / m->period, 0, 1);
    }
    const float accel = acceleration_limit(m);
    if (limits_for_motion()) {
        const float distance = m->goal - m->reference;
        const float remaining = fabsf(distance);
        const float half_step = 0.5f * accel * CONTROL_DT;
        const float stopping_speed = sqrtf(2 * accel * remaining + half_step * half_step) - half_step;
        const float desired_speed = copysignf(fminf(remaining / CONTROL_DT,
            fminf(speed_limit(m), stopping_speed)), distance);
        m->ref_velocity = approach(m->ref_velocity, desired_speed, accel * CONTROL_DT);
        const float minimum = ball_state == BALL_ACTIVE ? .01f * m->ball_min * m->span : 0;
        const float maximum = ball_state == BALL_ACTIVE ? .01f * m->ball_max * m->span : m->span;
        m->reference = clamp(m->reference + m->ref_velocity * CONTROL_DT, minimum, maximum);
    } else {
        m->reference = m->goal;
    }
}

static void position_update(Motor *m, float low, float high)
{
    const float error = m->reference - m->position;
    const float proportional = m->cfg.kp * error;
    m->derivative = m->alpha * m->derivative - m->beta * (m->position - m->previous_angle);
    const float step = m->ki_ts * error;
    const float base = proportional + m->derivative;
    const float candidate = base + m->integral + step;
    if (!((candidate > high && step > 0) || (candidate < low && step < 0))) {
        const float integral_limit = position_integral_limit(m);
        m->integral = clamp(m->integral + step, -integral_limit, integral_limit);
    }
    m->previous_angle = m->position;
    const float target = clamp(base + m->integral, low, high);
    m->velocity_target = limits_for_motion()
        ? approach(m->velocity_target, target, acceleration_limit(m) * CONTROL_DT) : target;
}

static void finish_level(LevelState state);
static void level_update(uint64_t now);
static void home_update(uint64_t now);
static void ball_center_update(uint64_t now);

static float pwm_test_update(unsigned i, uint64_t now)
{
    Motor *m = &motors[i];
    if (now - pwm_test.received_at > PWM_TEST_TIMEOUT_US
        || m->raw != 0 || m->position <= PWM_TEST_MARGIN_DEG
        || m->position >= m->span - PWM_TEST_MARGIN_DEG
        || fabsf(m->control_velocity) > m->cfg.speed) {
        stop_all(BRAKE, NO_FAULT);
        return 0;
    }
    const int64_t displacement = llabs(m->count - pwm_test.start_count);
    if (displacement == 0) {
        pwm_test.first_duty = 0;
    } else if (pwm_test.first_duty == 0 && m->output != 0) {
        pwm_test.first_duty = fabsf(m->output);
    }
    if (displacement >= 2) {
        if (pwm_test.first_duty > 0
            && (m->count - pwm_test.start_count) * encoder_direction(m) * pwm_test.direction > 0) {
            pwm_test.state = PWM_FOUND;
        }
        stop_all(BRAKE, NO_FAULT);
        return 0;
    }
    return pwm_test.direction * pwm_test.target;
}

static void update_motor(unsigned i, uint64_t now, bool control_sample)
{
    Motor *m = &motors[i];
    if (linked && now - last_heartbeat > LINK_TIMEOUT_US) {
        linked = false;
        motion_limits_enabled = true;
        stop_all(BRAKE, LINK_LOST);
    }
    if (ball_running() && now - ball_command_at >= BALL_TIMEOUT_US) {
        stop_all(BRAKE, NO_FAULT);
        ball_state = BALL_TIMEOUT;
    }
    m->raw = read_switches(&pins[i]);
    m->stable ^= (m->raw ^ m->stable) & ~(m->raw ^ m->previous_raw);
    m->previous_raw = m->raw;
    if (m->last_count != m->count) {
        m->last_count = m->count;
        m->encoder_motion_at = now;
    }
    const int sign = encoder_direction(m);
    const float speed = sign * m->control_velocity;
    m->position = m->travel.captured == LIMIT_BOTH
        ? (float)(m->count - m->travel.min_count) * ENCODER_DEGREES_PER_COUNT * sign
        : (float)m->count * ENCODER_DEGREES_PER_COUNT;

    float duty = 0;
    bool brake_at_zero = true;
    const unsigned other = i == MOTOR_1 ? MOTOR_3 : MOTOR_1;
    if (ball_state == BALL_ACTIVE && m->raw != 0) {
        if (m->ball_switch_at == 0) {
            m->ball_switch_at = now;
        }
    } else {
        m->ball_switch_at = 0;
    }
    if (m->raw == LIMIT_BOTH && (m->mode != BRAKE || m->fault != SWITCH_CONFLICT)) {
        trip_motor(i, SWITCH_CONFLICT);
    } else if (m->ball_switch_at != 0 && now - m->ball_switch_at >= BALL_SWITCH_CONFIRM_US) {
        trip_motor(i, BALL_ENDSTOP);
    } else if (ball_state == BALL_ACTIVE && (m->position < .01f * m->ball_min * m->span - POSITION_RELEASE_DEG
               || m->position > .01f * m->ball_max * m->span + POSITION_RELEASE_DEG)) {
        trip_motor(i, BALL_OUTSIDE_WINDOW);
    } else if (ball_state == BALL_ACTIVE && (m->raw != 0 || read_switches(&pins[other]) != 0)) {
        // Brake immediately on a raw switch; only latch the fault if it persists.
        neutral_output(i, true);
        neutral_output(other, true);
        reset_speed_pi(m);
        m->velocity_target = 0;
        m->guard_at = 0;
        m->guard_excess = false;
        m->stalled_at = 0;
    } else if (level_active() && m->raw != 0) {
        finish_level(LEVEL_TRAVEL);
    } else if (m->mode >= CALIBRATE
               && (m->sampled_at == 0 || now - m->sampled_at > 3u * SAMPLE_US)) {
        trip_motor(i, ENCODER_STALE);
    } else if (m->mode == CALIBRATE) {
        duty = calibration_update(i, now, control_sample);
        brake_at_zero = m->mode != CALIBRATE;
    } else if (m->mode == PWM_TEST) {
        duty = pwm_test_update(i, now);
    } else if (m->mode >= POSITION) {
        const bool fixed_goal = m->mode == POSITION || m->mode == STEP;
        const float minimum = ball_state == BALL_ACTIVE ? .01f * m->ball_min * m->span : 0;
        const float maximum = ball_state == BALL_ACTIVE ? .01f * m->ball_max * m->span : m->span;
        const float lower = fixed_goal ? fmaxf(minimum, m->goal) : minimum;
        const float upper = fixed_goal ? fminf(maximum, m->goal) : maximum;
        const float min_speed = limits_for_motion() ? speed_bound(m->position - lower, m) : INFINITY;
        const float max_speed = limits_for_motion() ? speed_bound(upper - m->position, m) : INFINITY;
        const float low = (m->raw & LIMIT_MIN) || m->position <= minimum ? 0 : -min_speed;
        const float high = (m->raw & LIMIT_MAX) || m->position >= maximum ? 0 : max_speed;
        if (m->mode == STEP) {
            m->step_elapsed = (now - m->started_at) * 1e-6f;
        }
        if (overspeed(m, speed_limit(m))) {
            trip_motor(i, OVERSPEED);
        } else if (m->position < -4 * ENCODER_DEGREES_PER_COUNT
                   || m->position > m->span + 4 * ENCODER_DEGREES_PER_COUNT) {
            trip_motor(i, OUT_OF_TRAVEL);
        } else if (((m->raw & LIMIT_MIN) && m->position > 4 * ENCODER_DEGREES_PER_COUNT)
            || ((m->raw & LIMIT_MAX) && m->position < m->span - 4 * ENCODER_DEGREES_PER_COUNT)) {
            invalidate_travel(m);
            trip_motor(i, UNEXPECTED_SWITCH);
        } else if (m->mode == STEP && now - m->started_at >= STEP_DURATION_US) {
            m->mode = BRAKE;
        } else {
            if (control_sample) {
                reference_update(m, now);
            }
            if (capturing_position(m, now)) {
                m->stalled_at = 0;
            } else {
                if (control_sample) {
                    position_update(m, low, high);
                }
                duty = speed_pi(m, now, control_sample, clamp(m->velocity_target, low, high), speed);
                brake_at_zero = m->position <= minimum || m->position >= maximum;
                if (stalled_at_limit(m, now)) {
                    trip_motor(i, STALLED);
                    duty = 0;
                }
            }
        }
    }
    bridge_output(i, duty, brake_at_zero, now);
    if (i == MOTOR_1 && control_sample) {
        if (level_active()) {
            level_update(now);
        } else if (home_state == HOME_MOVING) {
            home_update(now);
        } else if (ball_state == BALL_CENTERING) {
            ball_center_update(now);
        }
    }
    m->report_at = now;
    if (control_sample) {
        if (m->mode == BRAKE || m->mode == COAST) {
            m->goal = m->reference = m->position;
        }
        if (motors[MOTOR_1].report_at != 0 && now - motors[MOTOR_1].report_at < 100000u
            && motors[MOTOR_3].report_at != 0 && now - motors[MOTOR_3].report_at < 100000u
            && motors[MOTOR_1].sampled_at != 0 && now - motors[MOTOR_1].sampled_at < 100000u
            && motors[MOTOR_3].sampled_at != 0 && now - motors[MOTOR_3].sampled_at < 100000u) {
            watchdog_update();
        }
    }
}

void motor_task(void *parameter)
{
    const unsigned i = (unsigned)(uintptr_t)parameter;
    TickType_t wake = xTaskGetTickCount();
    unsigned divider = 0;
    for (;;) {
        xTaskDelayUntil(&wake, pdMS_TO_TICKS(ENCODER_PERIOD_MS));
        const bool control_sample = ++divider == CONTROL_MS / ENCODER_PERIOD_MS;
        if (control_sample) {
            divider = 0;
        }
        xSemaphoreTake(lock, portMAX_DELAY);
        update_motor(i, time_us_64(), control_sample);
        xSemaphoreGive(lock);
    }
}

unsigned motor_limits(void)
{
    xSemaphoreTake(lock, portMAX_DELAY);
    const unsigned limits = motors[MOTOR_1].stable | (motors[MOTOR_3].stable << 2);
    xSemaphoreGive(lock);
    return limits;
}

TravelCalibration motor_travel(unsigned i)
{
    xSemaphoreTake(lock, portMAX_DELAY);
    const TravelCalibration travel = motors[i].travel;
    xSemaphoreGive(lock);
    return travel;
}

void motor_link_down(void)
{
    xSemaphoreTake(lock, portMAX_DELAY);
    if (linked) {
        linked = false;
        motion_limits_enabled = true;
        stop_all(BRAKE, LINK_LOST);
    }
    xSemaphoreGive(lock);
}

static unsigned motor_number(unsigned i)
{
    return i == MOTOR_1 ? 1u : 3u;
}

static void print_settings(unsigned i, Settings cfg)
{
    printf("!CFG,%u,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%d,%u\n",
           motor_number(i), cfg.kp, cfg.ki, cfg.kd, cfg.tf, cfg.speed, cfg.acceleration,
           cfg.cal_fast, cfg.cal_slow, cfg.duty, cfg.polarity, (unsigned)cfg.law);
}

static bool valid_settings(const float *v)
{
    return v[0] >= 0 && v[0] <= 1000 && v[1] >= 0 && v[1] <= 5000
        && fabsf(v[2]) <= 100 && (v[10] != CONTROL_PIDF || v[2] == 0 || v[3] >= .0001f)
        && v[3] >= 0 && v[3] <= 2
        && v[4] >= 2 && v[4] <= 360 && v[5] >= 1 && v[5] <= 2000
        && v[6] >= v[7] && v[6] <= v[4] && v[7] >= 2
        && v[8] >= 0 && v[8] <= 100 && (v[9] == 1 || v[9] == -1)
        && (v[10] == CONTROL_P || v[10] == CONTROL_PI || v[10] == CONTROL_PID || v[10] == CONTROL_PIDF);
}

static const char *position_config_error(const Motor *m)
{
    if (m->cfg.duty == 0 || (m->cfg.kp == 0 && m->ki_ts == 0 && m->beta == 0)) {
        return "ganhos_ou_pwm_zerados";
    }
    return NULL;
}

static bool position_available(unsigned i)
{
    Motor *m = &motors[i];
    return m->travel.captured == LIMIT_BOTH && m->mode != COAST
        && read_switches(&pins[MOTOR_1]) != LIMIT_BOTH && read_switches(&pins[MOTOR_3]) != LIMIT_BOTH;
}

static void start_motion(Motor *m, Mode mode, float goal, float amplitude, float period, uint64_t now)
{
    if (home_state == HOME_BRAKED) {
        home_state = HOME_IDLE;
    }
    if (leveling.state == LEVEL_DONE) {
        leveling.state = LEVEL_IDLE;
    }
    reset_control(m);
    m->mode = mode;
    m->fault = NO_FAULT;
    m->started_at = now;
    m->encoder_motion_at = now;
    m->goal = goal;
    m->previous_goal_error = goal - m->position;
    m->baseline = mode == STEP ? goal - 1 : goal;
    m->amplitude = amplitude;
    m->period = period;
    m->step_elapsed = 0;
    m->step_id = mode == STEP ? ++next_step_id : 0;
}

static float home_goal(const Motor *m)
{
    return HOME_TRAVEL_FRACTION * m->span;
}

static void home_update(uint64_t now)
{
    if (now - home_started_at > HOME_TIMEOUT_US) {
        stop_all(BRAKE, NO_FAULT);
        home_state = HOME_TIMEOUT;
        return;
    }
    for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
        const Motor *m = &motors[i];
        if (m->sampled_at == 0 || now - m->sampled_at > 3u * SAMPLE_US) {
            trip_motor(i, ENCODER_STALE);
            return;
        }
        if (fabsf(home_goal(m) - m->position) > POSITION_RELEASE_DEG || m->output != 0
            || fabsf(m->velocity) >= ENCODER_DEGREES_PER_COUNT / .15f) {
            home_quiet_at = 0;
            return;
        }
    }
    if (home_quiet_at == 0) {
        home_quiet_at = now;
    } else if (now - home_quiet_at >= HOME_SETTLE_US) {
        stop_all(BRAKE, NO_FAULT);
        home_state = HOME_BRAKED;
        for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
            neutral_output(i, true);
        }
    }
}

static const char *begin_home(uint64_t now)
{
    for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
        const Motor *m = &motors[i];
        if (m->travel.captured != LIMIT_BOTH || m->span <= 0) {
            return "home_requer_curso";
        }
        if (read_switches(&pins[i]) == LIMIT_BOTH) {
            return "fins_simultaneos";
        }
        if (m->sampled_at == 0 || now - m->sampled_at > 3u * SAMPLE_US) {
            return "home_requer_encoder";
        }
        const char *error = position_config_error(m);
        if (error && fabsf(home_goal(m) - m->position) > POSITION_CAPTURE_DEG) {
            return error;
        }
    }
    stop_all(BRAKE, NO_FAULT);
    home_state = HOME_MOVING;
    home_started_at = now;
    home_quiet_at = 0;
    for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
        Motor *m = &motors[i];
        start_motion(m, POSITION, home_goal(m), 0, 0, now);
    }
    return NULL;
}

static void finish_level(LevelState state)
{
    for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
        leveling.result[i] = motors[i].position;
    }
    leveling.state = state;
    stop_all(BRAKE, NO_FAULT);
}

static void level_move(LevelState state, const float targets[MOTOR_COUNT], uint64_t now)
{
    leveling.state = state;
    leveling.stage_at = now;
    leveling.quiet_at = 0;
    leveling.samples = 0;
    leveling.sum = (Tilt){0};
    for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
        Motor *m = &motors[i];
        leveling.target[i] = targets[i];
        neutral_output(i, true);
        if (fabsf(targets[i] - m->position) > 2 * ENCODER_DEGREES_PER_COUNT) {
            start_motion(m, POSITION, targets[i], 0, 0, now);
        } else {
            m->mode = BRAKE;
            reset_control(m);
        }
    }
}

static void level_probe(unsigned index, LevelState next, uint64_t now)
{
    const Motor *m = &motors[index];
    float targets[MOTOR_COUNT] = {motors[MOTOR_1].position, motors[MOTOR_3].position};
    const float delta = m->position < m->span * .5f ? LEVEL_PROBE_DEG : -LEVEL_PROBE_DEG;
    targets[index] = clamp(m->position + delta, LEVEL_MARGIN_DEG, m->span - LEVEL_MARGIN_DEG);
    if (fabsf(targets[index] - m->position) < 2) {
        finish_level(LEVEL_TRAVEL);
        return;
    }
    level_move(next, targets, now);
}

static bool level_fit_axes(Tilt measured)
{
    const float a = leveling.probe_pos[1][0] - leveling.probe_pos[0][0];
    const float b = motors[0].position - leveling.probe_pos[1][0];
    const float c = leveling.probe_pos[1][1] - leveling.probe_pos[0][1];
    const float d = motors[1].position - leveling.probe_pos[1][1];
    const float det = a * d - b * c;
    if (fabsf(det) < 4) {
        return false;
    }
    const Tilt first = {leveling.probe_tilt[1].x - leveling.probe_tilt[0].x,
                        leveling.probe_tilt[1].y - leveling.probe_tilt[0].y};
    const Tilt second = {measured.x - leveling.probe_tilt[1].x, measured.y - leveling.probe_tilt[1].y};
    leveling.axes[0] = (Tilt){(d * first.x - c * second.x) / det, (d * first.y - c * second.y) / det};
    leveling.axes[1] = (Tilt){(a * second.x - b * first.x) / det, (a * second.y - b * first.y) / det};
    return true;
}

static void level_correct(Tilt measured, uint64_t now)
{
    const Tilt a = leveling.axes[0], b = leveling.axes[1];
    const float det = a.x * b.y - a.y * b.x;
    if (fabsf(det) < .002f || fabsf(det) < .20f * hypotf(a.x, a.y) * hypotf(b.x, b.y)) {
        finish_level(LEVEL_GEOMETRY);
        return;
    }
    if (leveling.corrections > 0 && leveling.tilt >= leveling.previous_error - .08f) {
        ++leveling.no_progress;
    } else {
        leveling.no_progress = 0;
    }
    if (leveling.corrections >= 10 || leveling.no_progress >= 2) {
        finish_level(LEVEL_DIVERGED);
        return;
    }
    const float delta[MOTOR_COUNT] = {(b.x * measured.y - b.y * measured.x) / det,
                                    (a.y * measured.x - a.x * measured.y) / det};
    const float scale = fminf(.70f, 5.0f / fmaxf(fabsf(delta[0]), fabsf(delta[1])));
    float targets[MOTOR_COUNT];
    bool moves = false;
    for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
        const Motor *m = &motors[i];
        const float low = fminf(m->position, LEVEL_MARGIN_DEG);
        const float high = fmaxf(m->position, m->span - LEVEL_MARGIN_DEG);
        targets[i] = clamp(m->position + scale * delta[i], low, high);
        moves |= fabsf(targets[i] - m->position) > 2 * ENCODER_DEGREES_PER_COUNT;
    }
    if (!moves) {
        finish_level(LEVEL_TRAVEL);
        return;
    }
    leveling.previous_error = leveling.tilt;
    ++leveling.corrections;
    level_move(LEVEL_CORRECT, targets, now);
}

static bool level_stationary(void)
{
    const float norm = sqrtf(level_imu.x * level_imu.x + level_imu.y * level_imu.y + level_imu.z * level_imu.z);
    if (level_imu.gyro > 1.5f || fabsf(norm - LOCAL_GRAVITY_MPS2) > .05f * LOCAL_GRAVITY_MPS2) {
        return false;
    }
    for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
        const Motor *m = &motors[i];
        if (fabsf(m->position - leveling.target[i]) > 3 * ENCODER_DEGREES_PER_COUNT
            || fabsf(m->velocity) >= ENCODER_DEGREES_PER_COUNT / .15f || m->output != 0) {
            return false;
        }
    }
    return true;
}

static void level_update(uint64_t now)
{
    if (level_imu.at == 0 || now - level_imu.at > LEVEL_IMU_MAX_AGE_US || level_imu.z <= 0) {
        finish_level(LEVEL_IMU);
        return;
    }
    if (now - leveling.stage_at > LEVEL_STAGE_US || now - leveling.started_at > LEVEL_TOTAL_US) {
        finish_level(LEVEL_TIMEOUT);
        return;
    }
    if (read_switches(&pins[MOTOR_1]) != 0 || read_switches(&pins[MOTOR_3]) != 0) {
        finish_level(LEVEL_TRAVEL);
        return;
    }
    for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
        if (motors[i].sampled_at == 0 || now - motors[i].sampled_at > 3u * SAMPLE_US) {
            trip_motor(i, ENCODER_STALE);
            return;
        }
    }
    if (!level_stationary()) {
        leveling.quiet_at = 0;
        leveling.samples = 0;
        leveling.sum = (Tilt){0};
        return;
    }
    if (leveling.quiet_at == 0) {
        leveling.quiet_at = now;
    }
    if (now - leveling.quiet_at < LEVEL_SETTLE_US || level_imu.at == leveling.consumed_at) {
        return;
    }
    leveling.consumed_at = level_imu.at;
    const Tilt measured = {atan2f(level_imu.x, level_imu.z) * RAD_TO_DEG,
                           atan2f(level_imu.y, level_imu.z) * RAD_TO_DEG};
    if (leveling.samples == 0) {
        leveling.first = measured;
    } else if (hypotf(measured.x - leveling.first.x, measured.y - leveling.first.y) > .5f) {
        leveling.quiet_at = 0;
        leveling.samples = 0;
        leveling.sum = (Tilt){0};
        return;
    }
    leveling.sum.x += measured.x;
    leveling.sum.y += measured.y;
    if (++leveling.samples < LEVEL_SAMPLES) {
        return;
    }
    const Tilt average = {leveling.sum.x / LEVEL_SAMPLES, leveling.sum.y / LEVEL_SAMPLES};
    leveling.tilt = hypotf(average.x, average.y);
    if (leveling.tilt <= LEVEL_TOLERANCE_DEG) {
        finish_level(LEVEL_DONE);
    } else if (leveling.tilt > 25) {
        finish_level(LEVEL_GEOMETRY);
    } else if (leveling.state == LEVEL_BASELINE || leveling.state == LEVEL_PROBE_M1) {
        const unsigned probe = leveling.state == LEVEL_BASELINE ? MOTOR_1 : MOTOR_3;
        leveling.probe_tilt[probe] = average;
        for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
            leveling.probe_pos[probe][i] = motors[i].position;
        }
        level_probe(probe, probe == MOTOR_1 ? LEVEL_PROBE_M1 : LEVEL_PROBE_M3, now);
    } else if (leveling.state == LEVEL_PROBE_M3 && !level_fit_axes(average)) {
        finish_level(LEVEL_GEOMETRY);
    } else {
        level_correct(average, now);
    }
}

static const char *begin_level(uint64_t now)
{
    if (!motion_limits_enabled) {
        return "nivelamento_requer_limites";
    }
    if (level_imu.at == 0 || now - level_imu.at > LEVEL_IMU_MAX_AGE_US) {
        return "imu_indisponivel";
    }
    if (level_imu.z <= 0) {
        return "eixo_z_invertido";
    }
    float targets[MOTOR_COUNT];
    for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
        const Motor *m = &motors[i];
        if (m->mode != BRAKE) {
            return "freio_necessario";
        }
        if (m->travel.captured != LIMIT_BOTH || m->span < 2 * LEVEL_MARGIN_DEG + LEVEL_PROBE_DEG) {
            return "nivelamento_requer_curso";
        }
        if (m->position < 0 || m->position > m->span) {
            return "nivelamento_fora_do_curso";
        }
        if (read_switches(&pins[i]) != 0) {
            return "libere_os_fins_de_curso";
        }
        const char *error = position_config_error(m);
        if (error) {
            return error;
        }
        targets[i] = m->position;
    }
    if (home_state == HOME_BRAKED) {
        home_state = HOME_IDLE;
    }
    leveling = (Leveling){.started_at = now, .tilt = -1};
    level_move(LEVEL_BASELINE, targets, now);
    return NULL;
}

static bool motors_stationary(void)
{
    for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
        const Motor *m = &motors[i];
        if ((m->mode != BRAKE && !(m->mode == POSITION && m->capture_at != 0))
            || m->output != 0 || fabsf(m->velocity) >= ENCODER_DEGREES_PER_COUNT / .15f) {
            return false;
        }
    }
    return true;
}

static void ball_center_update(uint64_t now)
{
    if (now - ball_center_started_at > HOME_TIMEOUT_US) {
        const unsigned i = fabsf(motors[MOTOR_1].position - home_goal(&motors[MOTOR_1]))
            >= fabsf(motors[MOTOR_3].position - home_goal(&motors[MOTOR_3])) ? MOTOR_1 : MOTOR_3;
        trip_motor(i, BALL_CENTER_TIMEOUT);
        return;
    }
    bool centered = motors_stationary();
    for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
        centered &= fabsf(motors[i].position - home_goal(&motors[i])) <= POSITION_RELEASE_DEG;
    }
    if (!centered) {
        ball_center_quiet_at = 0;
    } else if (ball_center_quiet_at == 0) {
        ball_center_quiet_at = now;
    } else if (now - ball_center_quiet_at >= HOME_SETTLE_US) {
        ball_state = BALL_ACTIVE;
    }
}

static const char *begin_ball(const float *limits, uint64_t now)
{
    if (!motors_stationary()) {
        return "freio_necessario";
    }
    for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
        const Motor *m = &motors[i];
        if (limits[3 * i] < BALL_TRAVEL_MIN || limits[3 * i] >= HOME_PERCENT
            || limits[3 * i + 1] <= HOME_PERCENT || limits[3 * i + 1] > BALL_TRAVEL_MAX) {
            return "faixa_bola_invalida";
        }
        if (limits[3 * i + 2] <= 0 || limits[3 * i + 2] > 100) {
            return "pwm_bola_invalido";
        }
        if (m->travel.captured != LIMIT_BOTH || m->span <= 0) {
            return "home_requer_curso";
        }
        if (m->sampled_at == 0 || now - m->sampled_at > 3u * SAMPLE_US) {
            return "home_requer_encoder";
        }
        if (read_switches(&pins[i]) == LIMIT_BOTH) {
            return "fins_simultaneos";
        }
        if (m->position < -POSITION_RELEASE_DEG || m->position > m->span + POSITION_RELEASE_DEG) {
            return "bola_fora_do_curso";
        }
        const char *error = position_config_error(m);
        if (error) {
            return error;
        }
    }
    stop_all(BRAKE, NO_FAULT);
    ball_fault.fault = NO_FAULT;
    ball_state = BALL_CENTERING;
    ball_command_at = now;
    ball_center_started_at = now;
    ball_center_quiet_at = 0;
    for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
        Motor *m = &motors[i];
        m->ball_min = limits[3 * i];
        m->ball_max = limits[3 * i + 1];
        m->ball_duty = limits[3 * i + 2];
        start_motion(m, POSITION, home_goal(m), 0, 0, now);
    }
    return NULL;
}

static const char *update_ball_targets(const float *percent, uint64_t now)
{
    if (!ball_running()) {
        return "bola_inativa";
    }
    if (now - ball_command_at >= BALL_TIMEOUT_US) {
        stop_all(BRAKE, NO_FAULT);
        ball_state = BALL_TIMEOUT;
        return "bola_inativa";
    }
    for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
        if (percent[i] < motors[i].ball_min || percent[i] > motors[i].ball_max) {
            trip_motor(i, BALL_INVALID_TARGET);
            return "bola_fora_da_faixa";
        }
    }
    ball_command_at = now;
    if (ball_state == BALL_CENTERING) {
        return NULL;
    }
    for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
        Motor *m = &motors[i];
        // The position controller already limits the reference speed and acceleration.
        const float target = (percent[i] / 100.0f) * m->span;
        if (target != m->goal) {
            m->goal = target;
            m->previous_goal_error = target - m->position;
        }
    }
    return NULL;
}

static const char *dispatch(const char *cmd, const float *v, size_t n, uint64_t now)
{
    if (strcmp(cmd, "STOP") == 0 && n == 0) {
        stop_all(BRAKE, NO_FAULT);
        return NULL;
    }
    if (!linked) {
        return "sem_sessao";
    }
    if (strcmp(cmd, "PWMVAL") == 0 && n == 2) {
        if (pwm_test.state == PWM_RUNNING && v[0] == motor_number(pwm_test.motor)) {
            pwm_test.target = clamp(v[1], 0, motors[pwm_test.motor].cfg.duty);
            pwm_test.received_at = now;
        }
        return NULL;
    }
    if (strcmp(cmd, "PING") == 0 && n == 0) {
        last_heartbeat = now;
        return NULL;
    }
    if (strcmp(cmd, "IMUGET") == 0 && n == 0) {
        return NULL;
    }
    if (strcmp(cmd, "BALL") == 0 && n == MOTOR_COUNT) {
        return update_ball_targets(v, now);
    }
    if (strcmp(cmd, "BALLSTATUS") == 0 && n == 0) {
        return NULL;
    }
    if (ball_running() && strcmp(cmd, "COAST") != 0) {
        return "bola_em_andamento";
    }
    if (home_state == HOME_MOVING && strcmp(cmd, "COAST") != 0) {
        return "home_em_andamento";
    }
    if (level_active()) {
        return "nivelamento_em_andamento";
    }
    if (pwm_test.state == PWM_RUNNING && strcmp(cmd, "COAST") != 0) {
        return "teste_pwm_em_andamento";
    }
    if (strcmp(cmd, "LEVEL") == 0 && n == 0) {
        return begin_level(now);
    }
    if (strcmp(cmd, "IMUZERO") == 0 && n == 0) {
        if (!motors_stationary() || !imu_capture_reference(now)) {
            return "imu_requer_mesa_parada";
        }
        leveling.state = LEVEL_IDLE;
        return NULL;
    }
    if (strcmp(cmd, "IMUCFG") == 0 && n == 3) {
        if (!motors_stationary()) {
            return "imu_requer_mesa_parada";
        }
        if (!imu_set_reference(v[0], v[1], v[2])) {
            return "referencia_imu_invalida";
        }
        leveling.state = LEVEL_IDLE;
        return NULL;
    }
    if (strcmp(cmd, "IMUYAW") == 0 && n == 0) {
        if (level_imu.at == 0 || now - level_imu.at > LEVEL_IMU_MAX_AGE_US) {
            return "imu_indisponivel";
        }
        imu_zero_heading();
        return NULL;
    }
    if (strcmp(cmd, "LIMITS") == 0 && n == 1 && (v[0] == 0 || v[0] == 1)) {
        if (v[0] == 0 && (motors[MOTOR_1].mode != BRAKE || motors[MOTOR_3].mode != BRAKE)) {
            return "freio_necessario";
        }
        if (motion_limits_enabled != (v[0] == 1)) {
            stop_all(BRAKE, NO_FAULT);
            motion_limits_enabled = v[0] == 1;
        }
        return NULL;
    }
    if (strcmp(cmd, "COAST") == 0 && n == 0) {
        if (read_switches(&pins[MOTOR_1]) == LIMIT_BOTH || read_switches(&pins[MOTOR_3]) == LIMIT_BOTH) {
            return "fins_simultaneos";
        }
        if (leveling.state == LEVEL_DONE) {
            leveling.state = LEVEL_IDLE;
        }
        stop_all(COAST, NO_FAULT);
        return NULL;
    }
    if (motors[MOTOR_1].mode == CALIBRATE || motors[MOTOR_3].mode == CALIBRATE) {
        return "calibracao_em_andamento";
    }
    if (strcmp(cmd, "HOME") == 0 && n == 0) {
        return begin_home(now);
    }
    if (strcmp(cmd, "BALLSTART") == 0) {
        return n == 3 * MOTOR_COUNT ? begin_ball(v, now) : "faixa_bola_invalida";
    }
    if (strcmp(cmd, "CAL") == 0 && (n == 0 || (n == 1 && v[0] == 3))) {
        if (read_switches(&pins[MOTOR_1]) != 0 || read_switches(&pins[MOTOR_3]) != 0) {
            return "libere_os_fins_de_curso";
        }
        if (motors[MOTOR_1].mode > BRAKE || motors[MOTOR_3].mode > BRAKE) {
            return "freio_necessario";
        }
        if ((n == 0 && motors[MOTOR_1].cfg.duty == 0) || motors[MOTOR_3].cfg.duty == 0) {
            return "pwm_zerado";
        }
        invalidate_travel(&motors[MOTOR_3]);
        leveling.state = LEVEL_IDLE;
        home_state = HOME_IDLE;
        motors[MOTOR_3].fault = NO_FAULT;
        begin_calibration(&motors[n == 0 ? MOTOR_1 : MOTOR_3], now);
        return NULL;
    }
    if (strcmp(cmd, "CIRCLE") == 0 && n == 2) {
        for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
            const char *error = position_config_error(&motors[i]);
            if (error) {
                return error;
            }
        }
        if (!position_available(MOTOR_1) || !position_available(MOTOR_3)) {
            return "calibracao_ou_freio_necessario";
        }
        if (v[0] <= 0 || v[1] < 0.2f || v[1] > 120
            || 2 * v[0] > fminf(motors[MOTOR_1].span, motors[MOTOR_3].span)) {
            return "teste_fora_da_faixa";
        }
        start_motion(&motors[MOTOR_1], SINE, motors[MOTOR_1].span * 0.5f, v[0], v[1], now);
        start_motion(&motors[MOTOR_3], COSINE, motors[MOTOR_3].span * 0.5f, v[0], v[1], now);
        return NULL;
    }
    if (n == 0 || (v[0] != 1 && v[0] != 3)) {
        return "comando_ou_motor_invalido";
    }
    const unsigned i = v[0] == 1 ? MOTOR_1 : MOTOR_3;
    Motor *m = &motors[i];
    if (strcmp(cmd, "PWMSTART") == 0 && n == 2 && (v[1] == 1 || v[1] == -1)) {
        if (!motors_stationary()) {
            return "freio_necessario";
        }
        if (m->travel.captured != LIMIT_BOTH || m->position <= PWM_TEST_MARGIN_DEG
            || m->position >= m->span - PWM_TEST_MARGIN_DEG || read_switches(&pins[i]) != 0) {
            return "teste_pwm_requer_curso";
        }
        if (m->sampled_at == 0 || now - m->sampled_at > 3u * SAMPLE_US) {
            return "home_requer_encoder";
        }
        if (m->cfg.duty == 0) {
            return "pwm_zerado";
        }
        stop_all(BRAKE, NO_FAULT);
        m->mode = PWM_TEST;
        m->fault = NO_FAULT;
        pwm_test.motor = i;
        pwm_test.direction = (int)v[1];
        pwm_test.start_count = m->count;
        pwm_test.received_at = now;
        pwm_test.target = pwm_test.first_duty = 0;
        pwm_test.state = PWM_RUNNING;
        return NULL;
    }
    if (strcmp(cmd, "FRICTION") == 0 && n == 3 && v[1] >= 0 && v[1] <= 100 && v[2] >= 0 && v[2] <= 100) {
        m->breakaway_forward = v[1];
        m->breakaway_reverse = v[2];
        return NULL;
    }
    if (strcmp(cmd, "CFG") == 0 && n == 12) {
        if (!valid_settings(v + 1)) {
            return "configuracao_fora_da_faixa";
        }
        const int polarity = (int)v[10];
        if (polarity != m->cfg.polarity) {
            if (m->mode >= CALIBRATE) {
                return "freio_necessario";
            }
            invalidate_travel(m);
            leveling.state = LEVEL_IDLE;
            if (home_state == HOME_BRAKED) {
                home_state = HOME_IDLE;
            }
        }
        const ControlLaw law = (ControlLaw)v[11];
        const bool gains_changed = m->cfg.kp != v[1] || m->cfg.ki != v[2]
            || m->cfg.kd != v[3] || m->cfg.tf != v[4] || m->cfg.law != law;
        m->cfg = (Settings){v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8], v[9], polarity, law};
        update_coefficients(m);
        if (m->mode >= POSITION && position_config_error(m)) {
            m->mode = BRAKE;
            neutral_output(i, true);
            reset_control(m);
        } else if (gains_changed) {
            m->integral = m->derivative = m->velocity_target = 0;
            reset_speed_pi(m);
            m->previous_angle = m->position;
        } else {
            const float integral_limit = position_integral_limit(m);
            m->integral = clamp(m->integral, -integral_limit, integral_limit);
            m->speed_integral = clamp(m->speed_integral, -m->cfg.duty, m->cfg.duty);
        }
        return NULL;
    }
    const char *config_error = position_config_error(m);
    if (config_error) {
        return config_error;
    }
    if (!position_available(i)) {
        return "calibracao_ou_freio_necessario";
    }
    if (strcmp(cmd, "GO") == 0 && n == 3 && (v[1] == 0 || v[1] == 1)) {
        const float target = v[1] == 0 ? v[2] : v[2] * m->span / 100;
        if (target < 0 || target > m->span) {
            return "alvo_fora_do_curso";
        }
        start_motion(m, POSITION, target, 0, 0, now);
        return NULL;
    }
    if (strcmp(cmd, "STEP") == 0 && n == 1) {
        if (m->mode != BRAKE || fabsf(m->velocity) >= 2 * ENCODER_DEGREES_PER_COUNT / 0.15f) {
            return "freio_e_motor_parado_necessarios";
        }
        const float target = m->position + 1;
        if (m->position < 0 || target > m->span) {
            return "alvo_fora_do_curso";
        }
        start_motion(m, STEP, target, 0, 0, now);
        return NULL;
    }
    if (strcmp(cmd, "WAVE") == 0 && n == 5 && (v[1] == 0 || v[1] == 1 || v[1] == 2)) {
        const Mode mode = v[1] == 0 ? SINE : v[1] == 1 ? COSINE : RAMP;
        const float lower = mode == RAMP ? fminf(v[2], v[2] + v[3]) : v[2] - fabsf(v[3]);
        const float upper = mode == RAMP ? fmaxf(v[2], v[2] + v[3]) : v[2] + fabsf(v[3]);
        if (lower < 0 || upper > m->span || v[4] < 0.2f || v[4] > 120) {
            return "teste_fora_da_faixa";
        }
        start_motion(m, mode, v[2], v[3], v[4], now);
        return NULL;
    }
    return "comando_invalido";
}

void motor_command(char *line)
{
    char *save = NULL;
    const char *cmd = strtok_r(line, " ", &save);
    if (cmd == NULL) {
        return;
    }
    float values[12];
    size_t n = 0;
    for (const char *arg = strtok_r(NULL, " ", &save); arg != NULL; arg = strtok_r(NULL, " ", &save)) {
        char *end;
        const float value = strtof(arg, &end);
        if (n == count_of(values) || *end != '\0' || !isfinite(value)) {
            printf("!ERR,%s,argumento_invalido\n", cmd);
            return;
        }
        values[n++] = value;
    }
    xSemaphoreTake(lock, portMAX_DELAY);
    const uint64_t now = time_us_64();
    if (strcmp(cmd, "HELLO") == 0 && n == 0) {
        stop_all(BRAKE, NO_FAULT);
        motion_limits_enabled = true;
        for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
            if (motors[i].fault == LINK_LOST) {
                motors[i].fault = NO_FAULT;
            }
        }
        linked = true;
        last_heartbeat = now;
        const Settings m1 = motors[MOTOR_1].cfg;
        const Settings m3 = motors[MOTOR_3].cfg;
        const HomeState home = home_state;
        ball_state = BALL_IDLE;
        xSemaphoreGive(lock);
        puts("!HELLO,12");
        printf("!SCALE,%.7g\n", ENCODER_DEGREES_PER_COUNT);
        puts("!LIMITS,1");
        printf("!HOME,%u\n", (unsigned)home);
        puts("!BALL,0");
        imu_send_reference();
        print_settings(MOTOR_1, m1);
        print_settings(MOTOR_3, m3);
        return;
    }
    if (linked && now - last_heartbeat > LINK_TIMEOUT_US) {
        linked = false;
        motion_limits_enabled = true;
        stop_all(BRAKE, LINK_LOST);
    }
    const char *error = dispatch(cmd, values, n, now);
    const bool cfg = !error && strcmp(cmd, "CFG") == 0;
    const unsigned i = cfg && values[0] == 3 ? MOTOR_3 : MOTOR_1;
    const Settings settings = motors[i].cfg;
    const bool limits_enabled = motion_limits_enabled;
    const BallState ball = ball_state;
    xSemaphoreGive(lock);
    if (error && strcmp(cmd, "CFG") == 0 && n >= 1 && (values[0] == 1 || values[0] == 3)) {
        printf("!ERR,CFG,%s,%u\n", error, (unsigned)values[0]);
    } else if (error) {
        printf("!ERR,%s,%s\n", cmd, error);
    } else if (cfg) {
        print_settings(i, settings);
    } else if (strcmp(cmd, "LIMITS") == 0) {
        printf("!LIMITS,%u\n", (unsigned)limits_enabled);
    } else if (strcmp(cmd, "BALLSTATUS") == 0) {
        printf("!BALL,%u\n", (unsigned)ball);
    } else if (strcmp(cmd, "IMUZERO") == 0 || strcmp(cmd, "IMUCFG") == 0 || strcmp(cmd, "IMUGET") == 0) {
        imu_send_reference();
    } else if (strcmp(cmd, "PING") != 0 && strcmp(cmd, "BALL") != 0 && strcmp(cmd, "PWMVAL") != 0) {
        printf("!OK,%s\n", cmd);
    }
}

void motor_send_updates(void)
{
    static uint64_t last_report[MOTOR_COUNT];
    for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
        xSemaphoreTake(lock, portMAX_DELAY);
        const Motor *m = &motors[i];
        const uint64_t at = m->report_at;
        const float position = m->position, goal = m->goal, reference = m->reference;
        const float span = m->span, duty = m->output;
        const float measured = m->mode >= POSITION ? m->control_velocity : m->velocity;
        const float speed = m->travel.captured == LIMIT_BOTH ? encoder_direction(m) * measured : measured;
        const unsigned mode = m->mode, fault = m->fault, travel = m->travel.captured, raw = m->raw;
        const unsigned phase = m->phase, endpoint = m->endpoint;
        const uint32_t step_id = m->step_id;
        const float elapsed = m->step_elapsed;
        const int enable_level = mode <= COAST ? (int)gpio_get(pins[i].enable) : -1;
        const float fault_speed = fault == OVERSPEED || fault == STALLED ? m->fault_speed : 0;
        const float fault_threshold = fault == OVERSPEED || fault == STALLED ? m->fault_threshold : 0;
        // Before calibration the encoder sign relative to the course may be unknown.
        const float speed_setpoint = mode == CALIBRATE ? m->ref_velocity : m->speed_setpoint;
        xSemaphoreGive(lock);
        if (at == last_report[i]) {
            continue;
        }
        last_report[i] = at;
        printf("@,%u,%" PRIu64 ",%u,%u,%u,%u,%u,%u,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%" PRIu32 ",%.4f,%d,%.2f,%.2f,%.2f\n",
               motor_number(i), at, mode, fault, travel, raw, phase, endpoint,
               position, goal, reference, speed, span, duty, step_id, elapsed, enable_level,
               fault_speed, fault_threshold, speed_setpoint);
    }
    static uint64_t last_level_report;
    static LevelState last_level_state;
    static HomeState last_home_state;
    static BallState last_ball_state;
    static PwmTestState last_pwm_state;
    xSemaphoreTake(lock, portMAX_DELAY);
    const LevelState state = leveling.state;
    const HomeState home = home_state;
    const BallState ball = ball_state;
    const BallFault fault = ball_fault;
    const PwmTestState pwm_state = pwm_test.state;
    const unsigned pwm_motor = motor_number(pwm_test.motor);
    const int pwm_direction = pwm_test.direction;
    const float pwm_duty = pwm_test.first_duty;
    const float tilt = state == LEVEL_IDLE ? -1 : leveling.tilt;
    const float m1 = leveling.result[MOTOR_1];
    const float m3 = leveling.result[MOTOR_3];
    const uint64_t now = time_us_64();
    xSemaphoreGive(lock);
    if (pwm_state != last_pwm_state) {
        last_pwm_state = pwm_state;
        printf("!PWMTEST,%u,%u,%d,%.2f\n", pwm_motor, (unsigned)pwm_state, pwm_direction, pwm_duty);
    }
    if (state != last_level_state || home != last_home_state || ball != last_ball_state
        || now - last_level_report >= 250000u) {
        last_level_report = now;
        last_level_state = state;
        last_home_state = home;
        last_ball_state = ball;
        printf("!LEVEL,%u,%.2f,%.2f,%.2f\n", (unsigned)state, tilt, m1, m3);
        printf("!HOME,%u\n", (unsigned)home);
        if (ball == BALL_MOTOR && fault.fault != NO_FAULT) {
            printf("!BALLF,%u,%u,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                   fault.motor, (unsigned)fault.fault, fault.position, fault.target,
                   fault.duty, fault.speed, fault.threshold);
        }
        printf("!BALL,%u\n", (unsigned)ball);
    }
}
