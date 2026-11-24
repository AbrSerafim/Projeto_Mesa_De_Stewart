#include <math.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "encoder_channel.pio.h"
#include "quadrature_encoder.pio.h"
#include "hardware/i2c.h"
#include "hardware/watchdog.h"
#include "pico/critical_section.h"
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "motor_control.h"

#define IMU_I2C i2c0
#define IMU_SDA 4
#define IMU_SCL 5
#define I2C_BAUD 100000u
#define I2C_TIMEOUT_US 5000u
#define ENCODER_SPEED_WINDOW_MS 200u
#define IMU_PERIOD_MS 20u
#define LOG_PERIOD_MS 40u
#define CALIBRATION_SAMPLES (2000u / IMU_PERIOD_MS)

#define CORE_0 (1u << 0)
#define CORE_1 (1u << 1)

#define DEG_TO_RAD 0.01745329252f
#define RAD_TO_DEG 57.29577951f
#define TILT_GAIN 2.0f

enum {
    REG_SMPLRT_DIV = 0x19,
    REG_CONFIG = 0x1a,
    REG_GYRO_CONFIG = 0x1b,
    REG_ACCEL_CONFIG = 0x1c,
    REG_ACCEL_XOUT_H = 0x3b,
    REG_PWR_MGMT_1 = 0x6b,
    REG_WHO_AM_I = 0x75,
};

typedef struct {
    float x, y, z;
} Vec3;

typedef struct {
    Vec3 accel;
    Vec3 gyro;
} Sample;

typedef struct {
    float w, x, y, z;
} Quaternion;

typedef enum {
    IMU_STARTING,
    IMU_CALIBRATING,
    IMU_READY,
    IMU_OFFLINE,
} ImuState;

typedef struct {
    uint pin_a, pin_b;
    int sign;
    uint quadrature_sm, channel_a_sm, channel_b_sm;
} Encoder;

typedef struct {
    int64_t count;
    uint64_t pulses_a, pulses_b;
    float angle_deg, rpm;
} EncoderReading;

typedef struct {
    int64_t count;
    uint64_t time_us;
} EncoderSpeedSample;

static Encoder encoders[MOTOR_COUNT] = {
    [MOTOR_1] = {.pin_a = 8, .pin_b = 9, .sign = 1},
    [MOTOR_3] = {.pin_a = 6, .pin_b = 7, .sign = 1},
};

typedef struct {
    EncoderReading motor[MOTOR_COUNT];
    Sample imu;
    Vec3 angles;
    ImuState imu_state;
} Telemetry;

static Telemetry telemetry;
static critical_section_t telemetry_lock;

static void init_encoders(void)
{
    pio_add_program(pio0, &quadrature_encoder_program);
    const uint offset = pio_add_program(pio1, &encoder_channel_program);
    for (uint i = 0; i < MOTOR_COUNT; ++i) {
        Encoder *encoder = &encoders[i];
        encoder->quadrature_sm = pio_claim_unused_sm(pio0, true);
        quadrature_encoder_program_init(pio0, encoder->quadrature_sm, encoder->pin_a, 0);

        const uint pins[] = {encoder->pin_a, encoder->pin_b};
        uint *machines[] = {&encoder->channel_a_sm, &encoder->channel_b_sm};
        for (uint channel = 0; channel < 2; ++channel) {
            const uint sm = pio_claim_unused_sm(pio1, true);
            *machines[channel] = sm;
            pio_sm_config config = encoder_channel_program_get_default_config(offset);
            sm_config_set_jmp_pin(&config, pins[channel]);
            // PIO1 samples the same input pad while PIO0 owns the GPIO mux.
            pio_sm_init(pio1, sm, offset, &config);
            pio_sm_set_enabled(pio1, sm, true);
        }
    }
}

static uint32_t fresh_pio_count(PIO pio, uint sm)
{
    // Discard snapshots queued since the last read and take one fresh sample.
    uint entries = pio_sm_get_rx_fifo_level(pio, sm);
    while (entries-- > 0) {
        (void)pio_sm_get(pio, sm);
    }
    return pio_sm_get_blocking(pio, sm);
}

static void encoder_task(void *parameter)
{
    const Encoder *encoder = parameter;
    const uint motor = (uint)(encoder - encoders);
    EncoderReading reading = {0};
    uint32_t previous = fresh_pio_count(pio0, encoder->quadrature_sm);
    uint32_t previous_a = fresh_pio_count(pio1, encoder->channel_a_sm);
    uint32_t previous_b = fresh_pio_count(pio1, encoder->channel_b_sm);
    const EncoderSpeedSample initial = {.time_us = time_us_64()};
    EncoderSpeedSample speed_history[ENCODER_SPEED_WINDOW_MS / ENCODER_PERIOD_MS] = {0};
    uint speed_slot = 0;
    TickType_t wake = xTaskGetTickCount();

    for (;;) {
        xTaskDelayUntil(&wake, pdMS_TO_TICKS(ENCODER_PERIOD_MS));
        const uint32_t position = fresh_pio_count(pio0, encoder->quadrature_sm);
        const uint32_t channel_a = fresh_pio_count(pio1, encoder->channel_a_sm);
        const uint32_t channel_b = fresh_pio_count(pio1, encoder->channel_b_sm);
        const uint64_t now_us = time_us_64();
        const uint32_t difference = position - previous;
        const int64_t delta = difference <= INT32_MAX
            ? (int64_t)difference : (int64_t)difference - (INT64_C(1) << 32);

        reading.count += delta;
        reading.pulses_a += previous_a - channel_a;
        reading.pulses_b += previous_b - channel_b;
        reading.angle_deg = (float)reading.count * encoder->sign * ENCODER_DEGREES_PER_COUNT;

        motor_encoder_sample(motor, reading.count, now_us);
        // Use a rolling window for display speed while publishing position every 5 ms.
        const EncoderSpeedSample old = speed_history[speed_slot].time_us != 0
            ? speed_history[speed_slot] : initial;
        const uint64_t elapsed_us = now_us - old.time_us;
        reading.rpm = elapsed_us > 0
            ? (float)(reading.count - old.count) * encoder->sign * 60000000.0f
                / (ENCODER_COUNTS_PER_OUTPUT_REV * (float)elapsed_us)
            : 0.0f;
        speed_history[speed_slot] = (EncoderSpeedSample){reading.count, now_us};
        speed_slot = (speed_slot + 1) % count_of(speed_history);

        critical_section_enter_blocking(&telemetry_lock);
        telemetry.motor[motor] = reading;
        critical_section_exit(&telemetry_lock);
        previous = position;
        previous_a = channel_a;
        previous_b = channel_b;
    }
}

static float travel_percent(const TravelCalibration *travel, int64_t count)
{
    const float percent = 100.0f * (float)(count - travel->min_count)
        / (float)(travel->max_count - travel->min_count);
    return fminf(100.0f, fmaxf(0.0f, percent));
}

static bool write_register(uint8_t address, uint8_t reg, uint8_t value)
{
    const uint8_t command[] = {reg, value};
    return i2c_write_timeout_us(IMU_I2C, address, command, sizeof command,
                                false, I2C_TIMEOUT_US) == sizeof command;
}

static bool read_registers(uint8_t address, uint8_t reg, uint8_t *data, size_t length)
{
    return i2c_write_timeout_us(IMU_I2C, address, &reg, 1, true, I2C_TIMEOUT_US) == 1
        && i2c_read_timeout_us(IMU_I2C, address, data, length, false, I2C_TIMEOUT_US)
           == (int)length;
}

static uint8_t init_sensor(void)
{
    for (uint8_t address = 0x68; address <= 0x69; ++address) {
        uint8_t identity;
        if (!read_registers(address, REG_WHO_AM_I, &identity, 1) || identity != 0x68) {
            continue;
        }
        if (!write_register(address, REG_PWR_MGMT_1, 0x80)) {
            continue;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
        if (write_register(address, REG_PWR_MGMT_1, 0x01) // PLL with X gyro reference
            && write_register(address, REG_CONFIG, 0x04) // DLPF: accel 21 Hz, gyro 20 Hz
            && write_register(address, REG_SMPLRT_DIV, 4) // 1 kHz / (1 + 4) = 200 Hz
            && write_register(address, REG_GYRO_CONFIG, 0x00) // +/-250 deg/s
            && write_register(address, REG_ACCEL_CONFIG, 0x00)) { // +/-2 g
            vTaskDelay(pdMS_TO_TICKS(100));
            return address;
        }
    }
    return 0;
}

static int16_t read_i16(const uint8_t *data)
{
    const int32_t value = ((int32_t)data[0] << 8) | data[1];
    return (int16_t)(value >= 32768 ? value - 65536 : value);
}

static bool read_sample(uint8_t address, Sample *sample)
{
    uint8_t raw[14];
    if (!read_registers(address, REG_ACCEL_XOUT_H, raw, sizeof raw)) {
        return false;
    }
    sample->accel = (Vec3){
        read_i16(raw) / 16384.0f,
        read_i16(raw + 2) / 16384.0f,
        read_i16(raw + 4) / 16384.0f,
    };
    // Bytes 6 and 7 contain temperature.
    sample->gyro = (Vec3){
        read_i16(raw + 8) / 131.0f,
        read_i16(raw + 10) / 131.0f,
        read_i16(raw + 12) / 131.0f,
    };
    return true;
}

static float magnitude(Vec3 vector)
{
    return sqrtf(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
}

static float distance(Vec3 a, Vec3 b)
{
    return magnitude((Vec3){a.x - b.x, a.y - b.y, a.z - b.z});
}

static bool calibrate(uint8_t address, Vec3 *bias, Vec3 *initial_accel)
{
    Vec3 gyro_sum = {0};
    Vec3 accel_sum = {0};
    Sample reference = {0};
    unsigned count = 0;
    TickType_t wake = xTaskGetTickCount();

    while (count < CALIBRATION_SAMPLES) {
        xTaskDelayUntil(&wake, pdMS_TO_TICKS(IMU_PERIOD_MS));

        Sample sample;
        if (!read_sample(address, &sample)) {
            return false;
        }
        if (count == 0) {
            reference = sample;
        }
        if (fabsf(magnitude(sample.accel) - 1.0f) > 0.15f
            || magnitude(sample.gyro) > 30.0f
            || distance(sample.gyro, reference.gyro) > 2.0f
            || distance(sample.accel, reference.accel) > 0.05f) {
            gyro_sum = (Vec3){0};
            accel_sum = (Vec3){0};
            count = 0;
            continue;
        }
        gyro_sum.x += sample.gyro.x;
        gyro_sum.y += sample.gyro.y;
        gyro_sum.z += sample.gyro.z;
        accel_sum.x += sample.accel.x;
        accel_sum.y += sample.accel.y;
        accel_sum.z += sample.accel.z;
        ++count;
    }

    *bias = (Vec3){gyro_sum.x / count, gyro_sum.y / count, gyro_sum.z / count};
    *initial_accel = accel_sum;
    return true;
}

static Quaternion initial_attitude(Vec3 accel)
{
    const float roll = atan2f(accel.y, accel.z);
    const float pitch = atan2f(-accel.x, hypotf(accel.y, accel.z));
    const float cr = cosf(roll * 0.5f);
    const float sr = sinf(roll * 0.5f);
    const float cp = cosf(pitch * 0.5f);
    const float sp = sinf(pitch * 0.5f);
    return (Quaternion){cr * cp, sr * cp, cr * sp, -sr * sp};
}

static Quaternion update_attitude(Quaternion q, Sample sample, float dt)
{
    Vec3 rate = {
        sample.gyro.x * DEG_TO_RAD,
        sample.gyro.y * DEG_TO_RAD,
        sample.gyro.z * DEG_TO_RAD,
    };

    // Compare the accelerometer with the estimated upward direction in sensor axes.
    // Reduce the correction as the measured acceleration moves away from 1 g.
    const float norm = magnitude(sample.accel);
    const float weight = 1.0f - fabsf(norm - 1.0f) / 0.20f;
    if (weight > 0.0f) {
        const Vec3 up = {
            2.0f * (q.x * q.z - q.w * q.y),
            2.0f * (q.y * q.z + q.w * q.x),
            1.0f - 2.0f * (q.x * q.x + q.y * q.y),
        };
        const Vec3 a = {sample.accel.x / norm, sample.accel.y / norm, sample.accel.z / norm};
        rate.x += TILT_GAIN * weight * (a.y * up.z - a.z * up.y);
        rate.y += TILT_GAIN * weight * (a.z * up.x - a.x * up.z);
        rate.z += TILT_GAIN * weight * (a.x * up.y - a.y * up.x);
    }

    // Integrate body-frame angular velocity and normalize the quaternion.
    const float half_dt = 0.5f * dt;
    const Quaternion next = {
        q.w - half_dt * (q.x * rate.x + q.y * rate.y + q.z * rate.z),
        q.x + half_dt * (q.w * rate.x + q.y * rate.z - q.z * rate.y),
        q.y + half_dt * (q.w * rate.y + q.z * rate.x - q.x * rate.z),
        q.z + half_dt * (q.w * rate.z + q.x * rate.y - q.y * rate.x),
    };
    const float scale = 1.0f / sqrtf(next.w * next.w + next.x * next.x
                                  + next.y * next.y + next.z * next.z);
    return (Quaternion){next.w * scale, next.x * scale, next.y * scale, next.z * scale};
}

static Vec3 euler_degrees(Quaternion q)
{
    const float sin_pitch = 2.0f * (q.w * q.y - q.z * q.x);
    // At gimbal lock, choose roll = 0 and encode the combined rotation in yaw.
    if (fabsf(sin_pitch) >= 0.9999999f) {
        const float yaw = 2.0f * atan2f(q.z, q.w);
        return (Vec3){0.0f, copysignf(90.0f, sin_pitch),
                      remainderf(yaw * RAD_TO_DEG, 360.0f)};
    }
    return (Vec3){
        RAD_TO_DEG * atan2f(2.0f * (q.w * q.x + q.y * q.z),
                           1.0f - 2.0f * (q.x * q.x + q.y * q.y)),
        RAD_TO_DEG * asinf(sin_pitch),
        RAD_TO_DEG * atan2f(2.0f * (q.w * q.z + q.x * q.y),
                           1.0f - 2.0f * (q.y * q.y + q.z * q.z)),
    };
}

static void publish_imu_state(ImuState state)
{
    critical_section_enter_blocking(&telemetry_lock);
    telemetry.imu_state = state;
    critical_section_exit(&telemetry_lock);
}

static void sample_imu(uint8_t address, Vec3 bias, Vec3 initial_accel)
{
    Quaternion attitude = initial_attitude(initial_accel);
    uint64_t last_sample_us = time_us_64();
    TickType_t wake = xTaskGetTickCount();

    while (true) {
        xTaskDelayUntil(&wake, pdMS_TO_TICKS(IMU_PERIOD_MS));
        Sample sample;
        const uint64_t now_us = time_us_64();
        if (!read_sample(address, &sample)) {
            return;
        }
        sample.gyro.x -= bias.x;
        sample.gyro.y -= bias.y;
        sample.gyro.z -= bias.z;
        attitude = update_attitude(attitude, sample, (now_us - last_sample_us) * 1e-6f);
        last_sample_us = now_us;

        const Vec3 angle = euler_degrees(attitude);
        critical_section_enter_blocking(&telemetry_lock);
        telemetry.imu = sample;
        telemetry.angles = angle;
        telemetry.imu_state = IMU_READY;
        critical_section_exit(&telemetry_lock);
    }
}

static void imu_task(void *parameter)
{
    (void)parameter;
    vTaskDelay(pdMS_TO_TICKS(100));
    while (true) {
        const uint8_t address = init_sensor();
        if (address != 0) {
            Vec3 bias;
            Vec3 initial_accel;
            publish_imu_state(IMU_CALIBRATING);
            if (calibrate(address, &bias, &initial_accel)) {
                sample_imu(address, bias, initial_accel);
            }
        }
        publish_imu_state(IMU_OFFLINE);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void log_telemetry(void)
{
    critical_section_enter_blocking(&telemetry_lock);
    const Telemetry snapshot = telemetry;
    critical_section_exit(&telemetry_lock);

    char imu_csv[112] = ",,,,,,,,";
    if (snapshot.imu_state == IMU_READY) {
        const Sample s = snapshot.imu;
        const Vec3 a = snapshot.angles;
        snprintf(imu_csv, sizeof imu_csv, "%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f",
                 s.accel.x, s.accel.y, s.accel.z, s.gyro.x, s.gyro.y, s.gyro.z, a.x, a.y, a.z);
    }
    const EncoderReading m1 = snapshot.motor[MOTOR_1];
    const EncoderReading m3 = snapshot.motor[MOTOR_3];
    const unsigned limits = motor_limits();
    TravelCalibration travel[MOTOR_COUNT];
    char travel_csv[MOTOR_COUNT][12] = {{0}};
    for (uint motor = 0; motor < MOTOR_COUNT; ++motor) {
        travel[motor] = motor_travel(motor);
        if (travel[motor].captured == LIMIT_BOTH) {
            snprintf(travel_csv[motor], sizeof travel_csv[motor], "%.2f",
                     travel_percent(&travel[motor], snapshot.motor[motor].count));
        }
    }
    printf("%s,%" PRId64 ",%" PRIu64 ",%" PRIu64 ",%.4f,%.3f,"
           "%" PRId64 ",%" PRIu64 ",%" PRIu64 ",%.4f,%.3f,%u,%u,%u,%u,%u,%u,%s,%u,%s\n",
           imu_csv, m1.count, m1.pulses_a, m1.pulses_b, m1.angle_deg, m1.rpm,
           m3.count, m3.pulses_a, m3.pulses_b, m3.angle_deg, m3.rpm,
           limits & 1u, (limits >> 1) & 1u,
           (limits >> 2) & 1u, (limits >> 3) & 1u,
           (unsigned)snapshot.imu_state,
           travel[MOTOR_1].captured, travel_csv[MOTOR_1],
           travel[MOTOR_3].captured, travel_csv[MOTOR_3]);
}

static void communication_task(void *parameter)
{
    (void)parameter;
    char line[192];
    size_t length = 0;
    bool discard = false;
    bool connected = false;
    uint64_t last_log = 0;
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        xTaskDelayUntil(&wake, pdMS_TO_TICKS(2));
        if (!stdio_usb_connected()) {
            if (connected) {
                motor_link_down();
            }
            connected = false;
            length = 0;
            discard = false;
            continue;
        }
        if (!connected) {
            connected = true;
            puts("accelx,accely,accelz,gyrox,gyroy,gyroz,angx,angy,angz,"
                 "m1_count,m1_pulses_a,m1_pulses_b,m1_angle_deg,m1_rpm,"
                 "m3_count,m3_pulses_a,m3_pulses_b,m3_angle_deg,m3_rpm,"
                 "m1_min,m1_max,m3_min,m3_max,imu_state,"
                 "m1_limits_captured,m1_travel_pct,m3_limits_captured,m3_travel_pct");
        }
        for (unsigned n = 0; n < 256; ++n) {
            const int c = getchar_timeout_us(0);
            if (c < 0) {
                break;
            }
            if (c == '\n') {
                if (!discard && length != 0) {
                    line[length] = '\0';
                    motor_command(line);
                }
                length = 0;
                discard = false;
            } else if (c != '\r' && !discard) {
                if (length == sizeof line - 1 || c < 32 || c > 126) {
                    discard = true;
                } else {
                    line[length++] = (char)c;
                }
            }
        }
        motor_send_updates();
        const uint64_t now = time_us_64();
        if (now - last_log >= LOG_PERIOD_MS * 1000u) {
            last_log = now;
            log_telemetry();
        }
    }
}

int main(void)
{
    motors_init();
    stdio_init_all();
    critical_section_init(&telemetry_lock);
    init_encoders();
    i2c_init(IMU_I2C, I2C_BAUD);
    gpio_set_function(IMU_SDA, GPIO_FUNC_I2C);
    gpio_set_function(IMU_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_SDA);
    gpio_pull_up(IMU_SCL);

    if (xTaskCreateAffinitySet(encoder_task, "encoder_m1", 384, &encoders[MOTOR_1], 5, CORE_0, NULL) != pdPASS
        || xTaskCreateAffinitySet(encoder_task, "encoder_m3", 384, &encoders[MOTOR_3], 5, CORE_0, NULL) != pdPASS
        || xTaskCreateAffinitySet(motor_task, "motor_m1", 512, (void *)(uintptr_t)MOTOR_1, 4, CORE_0, NULL) != pdPASS
        || xTaskCreateAffinitySet(motor_task, "motor_m3", 512, (void *)(uintptr_t)MOTOR_3, 4, CORE_0, NULL) != pdPASS
        || xTaskCreateAffinitySet(imu_task, "imu", 512, NULL, 2, CORE_0, NULL) != pdPASS
        || xTaskCreateAffinitySet(communication_task, "usb_serial", 1024, NULL, 2, CORE_1, NULL) != pdPASS) {
        panic("Could not allocate FreeRTOS tasks");
    }
    watchdog_enable(1000, true);
    vTaskStartScheduler();
    panic("FreeRTOS scheduler stopped");
}
