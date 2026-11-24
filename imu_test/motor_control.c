#include "motor_control.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "hardware/pwm.h"
#include "hardware/watchdog.h"
#include "pico/stdlib.h"

#define CONTROL_MS 20u
#define CONTROL_DT (CONTROL_MS / 1000.0f)
#define SAMPLE_US (ENCODER_PERIOD_MS * 1000u)
#define HISTORY_SAMPLES 30u
#define LINK_TIMEOUT_US 1000000u
#define PHASE_TIMEOUT_US 45000000u
#define STALL_TIMEOUT_US 4000000u
#define STEP_DURATION_US 6000000u
#define PWM_TOP 6249u
#define PWM_RISE_PER_TICK (25.0f * ENCODER_PERIOD_MS / 1000.0f)
#define REVERSAL_US 30000u
#define CAL_PAUSE_US 80000u
#define SPEED_GUARD_WINDOW_US 150000u
#define SPEED_PI_PERIOD_US 50000u
// Reference PI of 1.20 and 0.90 per rpm, converted to degrees/second.
#define SPEED_PI_KP 0.20f
#define SPEED_PI_KI 0.15f
// Duty that overcame static friction in the earlier position bench test.
#define FRICTION_DUTY_PERCENT 13.0f
#define FRICTION_BLEND_SPEED 1.0f
#define POSITION_CAPTURE_DEG ENCODER_DEGREES_PER_COUNT
#define POSITION_RELEASE_DEG (2 * ENCODER_DEGREES_PER_COUNT)
#define POSITION_BRAKE_US 150000u
#define POSITION_SPEED_DELAY_S 0.10f

typedef struct {
    uint in_positive, in_negative, enable, min_switch, max_switch;
} Pins;

static const Pins pins[MOTOR_COUNT] = {
    {2, 3, 16, 10, 11},
    {15, 14, 17, 12, 13},
};

typedef struct {
    float kp, ki, kd, tf;
    float speed, acceleration, cal_fast, cal_slow, duty;
    int polarity;
} Settings;

typedef enum { BRAKE, COAST, CALIBRATE, POSITION, STEP, SINE, COSINE, RAMP } Mode;
typedef enum { SEEK_FAST, BACK_OFF, SEEK_SLOW, CLEAR_SWITCH } Phase;
typedef enum {
    NO_FAULT, LINK_LOST, SWITCH_CONFLICT, WRONG_SWITCH, CAL_TIMEOUT,
    ENCODER_STALE, STALLED, INVALID_SPAN, UNEXPECTED_SWITCH, OVERSPEED, OUT_OF_TRAVEL,
} Fault;

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
    uint64_t sampled_at, phase_at, moved_at, started_at, report_at, drive_at;
    uint64_t guard_at, speed_pi_at;
    uint64_t capture_at;
    int64_t guard_count;
    bool guard_excess;
    uint32_t step_id;
    float step_elapsed;
    float velocity, position, goal, reference, ref_velocity, output, requested;
    float fault_speed, fault_threshold;
    float previous_goal_error;
    float velocity_target, speed_setpoint, speed_integral;
    int speed_direction;
    float integral, derivative, previous_angle, ki_ts, alpha, beta;
    float baseline, amplitude, period;
    float span;
} Motor;

static Motor motors[MOTOR_COUNT] = {
    {.cfg = {4, 0, 0, .05f, 30, 90, 8, 4, 30, 1}},
    {.cfg = {4, 0, 0, .05f, 30, 90, 8, 4, 30, 1}},
};

static SemaphoreHandle_t lock;
static uint64_t last_heartbeat;
static bool linked;
static uint32_t next_step_id;

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

static void set_input_pwm(const Pins *p, int direction, uint16_t level)
{
    const uint pin = direction > 0 ? p->in_positive : p->in_negative;
    const bool channel_a = pwm_gpio_to_channel(pin) == PWM_CHAN_A;
    pwm_set_both_levels(pwm_gpio_to_slice_num(pin), channel_a ? level : 0, channel_a ? 0 : level);
}

static void neutral_output(unsigned i, bool brake)
{
    Motor *m = &motors[i];
    const Pins *p = &pins[i];
    if (!brake) {
        gpio_put(p->enable, 0);
    }
    set_input_pwm(p, 0, 0);
    if (brake) {
        gpio_put(p->enable, 1);
    }
    m->output = 0;
    m->last_direction = 0;
}

static void bridge_output(unsigned i, float duty, uint64_t now)
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
    float target = duty;
    if (m->output * duty < 0) {
        target = 0;
    }
    if (m->output == 0 && direction != 0 && direction != m->last_direction) {
        set_input_pwm(p, 0, 0);
        gpio_put(p->enable, 1);
        m->drive_at = now + (m->last_direction == 0 ? SAMPLE_US : REVERSAL_US);
        m->last_direction = direction;
        return;
    }
    if (m->output == 0 && (direction == 0 || now < m->drive_at)) {
        set_input_pwm(p, 0, 0);
        gpio_put(p->enable, 1);
        return;
    }
    const float limited = m->mode == CALIBRATE && fabsf(target) > fabsf(m->output)
        ? approach(m->output, target, PWM_RISE_PER_TICK) : target;
    const float next = clamp(limited, -m->cfg.duty, m->cfg.duty);
    const uint16_t level = (uint16_t)roundf(fabsf(next) * (PWM_TOP + 1) / 100.0f);
    set_input_pwm(p, m->last_direction, level);
    gpio_put(p->enable, 1);
    m->output = level == 0 ? 0 : next;
}

static void update_coefficients(Motor *m)
{
    m->ki_ts = m->cfg.ki * CONTROL_DT;
    m->alpha = m->cfg.tf / (m->cfg.tf + CONTROL_DT);
    m->beta = m->cfg.kd / (m->cfg.tf + CONTROL_DT);
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
    m->guard_excess = false;
    m->capture_at = 0;
}

static void invalidate_travel(Motor *m)
{
    m->travel = (TravelCalibration){0};
    m->span = 0;
}

static void stop_all(Mode mode, Fault fault)
{
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
    motors[i].fault = fault;
    stop_all(BRAKE, NO_FAULT);
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
        const uint slice = pwm_gpio_to_slice_num(p->in_positive);
        pwm_config config = pwm_get_default_config();
        pwm_config_set_wrap(&config, PWM_TOP);
        pwm_init(slice, &config, false);
        set_input_pwm(p, 0, 0);
        gpio_set_function(p->in_positive, GPIO_FUNC_PWM);
        gpio_set_function(p->in_negative, GPIO_FUNC_PWM);
        pwm_set_enabled(slice, true);
        update_coefficients(&motors[i]);
    }
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
    m->moved_at = now;
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
    if (now - m->moved_at > STALL_TIMEOUT_US) {
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
        m->ref_velocity = approach(m->ref_velocity, speed, m->cfg.acceleration * CONTROL_DT);
        const float error = m->ref_velocity - fabsf(m->velocity);
        const float next_i = clamp(m->integral + 3.0f * CONTROL_DT * error, 0, m->cfg.duty);
        const float proportional = .75f * error;
        if (proportional + next_i <= m->cfg.duty || error < 0) {
            m->integral = next_i;
        }
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
    const bool crossed = error * m->previous_goal_error < 0;
    m->previous_goal_error = error;
    if (m->capture_at != 0) {
        if (fabsf(error) <= POSITION_RELEASE_DEG || now - m->capture_at < POSITION_BRAKE_US
            || fabsf(m->velocity) >= ENCODER_DEGREES_PER_COUNT / .15f) {
            return true;
        }
        m->capture_at = 0;
        m->reference = m->position;
        m->previous_angle = m->position;
        return false;
    }
    if (!crossed && fabsf(error) > POSITION_CAPTURE_DEG) {
        return false;
    }
    m->capture_at = now;
    m->ref_velocity = m->integral = m->derivative = m->velocity_target = 0;
    reset_speed_pi(m);
    return true;
}

static float speed_bound(float distance, const Motor *m)
{
    const float reaction = m->cfg.acceleration * POSITION_SPEED_DELAY_S;
    return fminf(m->cfg.speed,
                 sqrtf(2 * m->cfg.acceleration * fmaxf(0, distance) + reaction * reaction) - reaction);
}

static float speed_pi(Motor *m, uint64_t now, float target, float measured)
{
    if (fabsf(target) < .05f) {
        reset_speed_pi(m);
        return 0;
    }
    const int direction = target > 0 ? 1 : -1;
    if (direction != m->speed_direction) {
        reset_speed_pi(m);
        m->speed_direction = direction;
    }
    m->speed_setpoint = target;
    if (m->speed_pi_at != 0 && now - m->speed_pi_at < SPEED_PI_PERIOD_US) {
        return m->requested;
    }
    const float dt = m->speed_pi_at == 0 ? 0 : (now - m->speed_pi_at) * 1e-6f;
    m->speed_pi_at = now;
    const float speed = fabsf(target);
    const float error = speed - direction * measured;
    const float friction = fminf(FRICTION_DUTY_PERCENT, m->cfg.duty * .60f);
    const float feedforward = friction * fminf(1, speed / FRICTION_BLEND_SPEED);
    const float base = feedforward + SPEED_PI_KP * error;
    const float step = SPEED_PI_KI * dt * error;
    const float candidate = base + m->speed_integral + step;
    if (!((candidate > m->cfg.duty && step > 0) || (candidate < 0 && step < 0))) {
        m->speed_integral = clamp(m->speed_integral + step, -friction, m->cfg.duty);
    }
    m->requested = direction * clamp(base + m->speed_integral, 0, m->cfg.duty);
    return m->requested;
}

static void position_update(Motor *m, uint64_t now, float low, float high)
{
    const float elapsed = (now - m->started_at) * 1e-6f;
    if (m->mode == SINE || m->mode == COSINE) {
        const float phase = 6.28318530718f * fmodf(elapsed, m->period) / m->period;
        m->goal = m->baseline + m->amplitude * (m->mode == SINE ? sinf(phase) : cosf(phase));
    } else if (m->mode == RAMP) {
        m->goal = m->baseline + m->amplitude * clamp(elapsed / m->period, 0, 1);
    }
    const float distance = m->goal - m->reference;
    const float half_step = 0.5f * m->cfg.acceleration * CONTROL_DT;
    const float desired_speed = copysignf(fminf(m->cfg.speed,
        sqrtf(2 * m->cfg.acceleration * fabsf(distance) + half_step * half_step) - half_step), distance);
    m->ref_velocity = approach(m->ref_velocity, desired_speed, m->cfg.acceleration * CONTROL_DT);
    m->reference = clamp(m->reference + m->ref_velocity * CONTROL_DT, 0, m->span);
    const float error = m->reference - m->position;
    const float proportional = m->cfg.kp * error;
    m->derivative = m->alpha * m->derivative - m->beta * (m->position - m->previous_angle);
    const float step = m->ki_ts * error;
    const float base = proportional + m->derivative;
    const float candidate = base + m->integral + step;
    if (!((candidate > high && step > 0) || (candidate < low && step < 0))) {
        const float integral_limit = .25f * m->cfg.speed;
        m->integral = clamp(m->integral + step, -integral_limit, integral_limit);
    }
    m->previous_angle = m->position;
    const float target = clamp(base + m->integral, low, high);
    m->velocity_target = approach(m->velocity_target, target, m->cfg.acceleration * CONTROL_DT);
}

static void update_motor(unsigned i, uint64_t now, bool control_sample)
{
    Motor *m = &motors[i];
    if (linked && now - last_heartbeat > LINK_TIMEOUT_US) {
        linked = false;
        stop_all(BRAKE, LINK_LOST);
    }
    m->raw = read_switches(&pins[i]);
    m->stable ^= (m->raw ^ m->stable) & ~(m->raw ^ m->previous_raw);
    m->previous_raw = m->raw;
    if (m->last_count != m->count) {
        m->last_count = m->count;
        m->moved_at = now;
    }
    const int sign = encoder_direction(m);
    const float speed = sign * m->velocity;
    m->position = m->travel.captured == LIMIT_BOTH
        ? (float)(m->count - m->travel.min_count) * ENCODER_DEGREES_PER_COUNT * sign
        : (float)m->count * ENCODER_DEGREES_PER_COUNT;

    float duty = 0;
    if (m->raw == LIMIT_BOTH && (m->mode != BRAKE || m->fault != SWITCH_CONFLICT)) {
        trip_motor(i, SWITCH_CONFLICT);
    } else if (m->mode >= CALIBRATE
               && (m->sampled_at == 0 || now - m->sampled_at > 3u * SAMPLE_US)) {
        trip_motor(i, ENCODER_STALE);
    } else if (m->mode == CALIBRATE) {
        duty = calibration_update(i, now, control_sample);
    } else if (m->mode >= POSITION) {
        const bool fixed_goal = m->mode == POSITION || m->mode == STEP;
        const float lower = fixed_goal ? m->goal : 0;
        const float upper = fixed_goal ? m->goal : m->span;
        const float min_speed = speed_bound(m->position - lower, m);
        const float max_speed = speed_bound(upper - m->position, m);
        const float low = (m->raw & LIMIT_MIN) ? 0 : -min_speed;
        const float high = (m->raw & LIMIT_MAX) ? 0 : max_speed;
        if (m->mode == STEP && control_sample) {
            m->step_elapsed = (now - m->started_at) * 1e-6f;
        }
        if (overspeed(m, m->cfg.speed)) {
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
        } else if (capturing_position(m, now)) {
            neutral_output(i, true);
            m->moved_at = now;
        } else {
            if (control_sample) {
                position_update(m, now, low, high);
            }
            duty = speed_pi(m, now, clamp(m->velocity_target, low, high), speed);
            if (fabsf(m->output) > 5 && fabsf(m->goal - m->position) > ENCODER_DEGREES_PER_COUNT
                && now - m->moved_at > STALL_TIMEOUT_US) {
                trip_motor(i, STALLED);
                duty = 0;
            } else if (fabsf(m->output) <= 5 || fabsf(m->goal - m->position) <= ENCODER_DEGREES_PER_COUNT) {
                m->moved_at = now;
            }
        }
    }
    bridge_output(i, duty, now);
    if (control_sample) {
        if (m->mode == BRAKE || m->mode == COAST) {
            m->goal = m->reference = m->position;
        }
        m->report_at = now;
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
    printf("!CFG,%u,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%d\n",
           motor_number(i), cfg.kp, cfg.ki, cfg.kd, cfg.tf, cfg.speed, cfg.acceleration,
           cfg.cal_fast, cfg.cal_slow, cfg.duty, cfg.polarity);
}

static bool valid_settings(const float *v)
{
    return v[0] >= 0 && v[0] <= 1000 && v[1] >= 0 && v[1] <= 5000
        && fabsf(v[2]) <= 100 && (v[2] == 0 || v[3] >= 0.0001f) && v[3] >= 0 && v[3] <= 2
        && v[4] >= 2 && v[4] <= 360 && v[5] >= 1 && v[5] <= 2000
        && v[6] >= v[7] && v[6] <= v[4] && v[7] >= 2
        && v[8] >= 0 && v[8] <= 100 && (v[9] == 1 || v[9] == -1);
}

static const char *position_config_error(const Motor *m)
{
    if (m->cfg.duty == 0 || (m->cfg.kp == 0 && m->cfg.ki == 0 && m->cfg.kd == 0)) {
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
    reset_control(m);
    m->mode = mode;
    m->fault = NO_FAULT;
    m->started_at = now;
    m->moved_at = now;
    m->goal = goal;
    m->previous_goal_error = goal - m->position;
    m->baseline = mode == STEP ? goal - 1 : goal;
    m->amplitude = amplitude;
    m->period = period;
    m->step_elapsed = 0;
    m->step_id = mode == STEP ? ++next_step_id : 0;
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
    if (strcmp(cmd, "PING") == 0 && n == 0) {
        last_heartbeat = now;
        return NULL;
    }
    if (strcmp(cmd, "COAST") == 0 && n == 0) {
        if (read_switches(&pins[MOTOR_1]) == LIMIT_BOTH || read_switches(&pins[MOTOR_3]) == LIMIT_BOTH) {
            return "fins_simultaneos";
        }
        stop_all(COAST, NO_FAULT);
        return NULL;
    }
    if (motors[MOTOR_1].mode == CALIBRATE || motors[MOTOR_3].mode == CALIBRATE) {
        return "calibracao_em_andamento";
    }
    if (strcmp(cmd, "CAL") == 0 && n == 0) {
        if (read_switches(&pins[MOTOR_1]) != 0 || read_switches(&pins[MOTOR_3]) != 0) {
            return "libere_os_fins_de_curso";
        }
        if (motors[MOTOR_1].mode > BRAKE || motors[MOTOR_3].mode > BRAKE) {
            return "freio_necessario";
        }
        if (motors[MOTOR_1].cfg.duty == 0 || motors[MOTOR_3].cfg.duty == 0) {
            return "pwm_zerado";
        }
        invalidate_travel(&motors[MOTOR_3]);
        motors[MOTOR_3].fault = NO_FAULT;
        begin_calibration(&motors[MOTOR_1], now);
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
    if (strcmp(cmd, "CFG") == 0 && n == 11) {
        if (!valid_settings(v + 1)) {
            return "configuracao_fora_da_faixa";
        }
        const int polarity = (int)v[10];
        if (polarity != m->cfg.polarity) {
            if (m->mode >= CALIBRATE) {
                return "freio_necessario";
            }
            invalidate_travel(m);
        }
        const bool gains_changed = m->cfg.kp != v[1] || m->cfg.ki != v[2]
            || m->cfg.kd != v[3] || m->cfg.tf != v[4];
        m->cfg = (Settings){v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8], v[9], polarity};
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
            const float integral_limit = .25f * m->cfg.speed;
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
    float values[11];
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
        for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
            if (motors[i].fault == LINK_LOST) {
                motors[i].fault = NO_FAULT;
            }
        }
        linked = true;
        last_heartbeat = now;
        const Settings m1 = motors[MOTOR_1].cfg;
        const Settings m3 = motors[MOTOR_3].cfg;
        xSemaphoreGive(lock);
        puts("!HELLO,5");
        printf("!SCALE,%.7g\n", ENCODER_DEGREES_PER_COUNT);
        print_settings(MOTOR_1, m1);
        print_settings(MOTOR_3, m3);
        return;
    }
    if (linked && now - last_heartbeat > LINK_TIMEOUT_US) {
        linked = false;
        stop_all(BRAKE, LINK_LOST);
    }
    const char *error = dispatch(cmd, values, n, now);
    const bool cfg = !error && strcmp(cmd, "CFG") == 0;
    const unsigned i = cfg && values[0] == 3 ? MOTOR_3 : MOTOR_1;
    const Settings settings = motors[i].cfg;
    xSemaphoreGive(lock);
    if (error && strcmp(cmd, "CFG") == 0 && n >= 1 && (values[0] == 1 || values[0] == 3)) {
        printf("!ERR,CFG,%s,%u\n", error, (unsigned)values[0]);
    } else if (error) {
        printf("!ERR,%s,%s\n", cmd, error);
    } else if (cfg) {
        print_settings(i, settings);
    } else if (strcmp(cmd, "PING") != 0) {
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
        const float speed = m->travel.captured == LIMIT_BOTH ? encoder_direction(m) * m->velocity : m->velocity;
        const unsigned mode = m->mode, fault = m->fault, travel = m->travel.captured, raw = m->raw;
        const unsigned phase = m->phase, endpoint = m->endpoint;
        const uint32_t step_id = m->step_id;
        const float elapsed = m->step_elapsed;
        const int enable_level = mode <= COAST ? (int)gpio_get(pins[i].enable) : -1;
        const float fault_speed = fault == OVERSPEED ? m->fault_speed : 0;
        const float fault_threshold = fault == OVERSPEED ? m->fault_threshold : 0;
        const float speed_setpoint = mode >= POSITION ? m->speed_setpoint : 0;
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
}
