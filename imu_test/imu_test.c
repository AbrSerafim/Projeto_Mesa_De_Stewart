#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "hardware/i2c.h"
#include "pico/stdlib.h"

#define IMU_I2C i2c0
#define IMU_SDA 4
#define IMU_SCL 5
#define I2C_BAUD 100000u
#define I2C_TIMEOUT_US 5000u
#define SAMPLE_PERIOD_US 10000u
#define CALIBRATION_SAMPLES 200u

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
        sleep_ms(100);
        if (write_register(address, REG_PWR_MGMT_1, 0x01) // PLL with X gyro reference
            && write_register(address, REG_CONFIG, 0x04) // DLPF: accel 21 Hz, gyro 20 Hz
            && write_register(address, REG_SMPLRT_DIV, 4) // 1 kHz / (1 + 4) = 200 Hz
            && write_register(address, REG_GYRO_CONFIG, 0x00) // +/-250 deg/s
            && write_register(address, REG_ACCEL_CONFIG, 0x00)) { // +/-2 g
            sleep_ms(100);
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
    unsigned ticks = 0;
    absolute_time_t next = get_absolute_time();

    while (count < CALIBRATION_SAMPLES) {
        if (ticks++ % 100 == 0) {
            puts("# Calibrando o giroscopio. Mantenha a IMU parada.");
        }
        next = delayed_by_us(next, SAMPLE_PERIOD_US);
        sleep_until(next);

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

static void stream_samples(uint8_t address, Vec3 bias, Vec3 initial_accel)
{
    Quaternion attitude = initial_attitude(initial_accel);
    absolute_time_t next = get_absolute_time();
    uint64_t last_sample_us = to_us_since_boot(next);

    while (true) {
        next = delayed_by_us(next, SAMPLE_PERIOD_US);
        if (time_reached(next)) {
            next = delayed_by_us(get_absolute_time(), SAMPLE_PERIOD_US);
        }
        sleep_until(next);

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
        printf("%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f\n",
               sample.accel.x, sample.accel.y, sample.accel.z,
               sample.gyro.x, sample.gyro.y, sample.gyro.z,
               angle.x, angle.y, angle.z);
    }
}

int main(void)
{
    stdio_init_all();
    i2c_init(IMU_I2C, I2C_BAUD);
    gpio_set_function(IMU_SDA, GPIO_FUNC_I2C);
    gpio_set_function(IMU_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_SDA);
    gpio_pull_up(IMU_SCL);
    sleep_ms(100);

    while (true) {
        const uint8_t address = init_sensor();
        if (address == 0) {
            puts("# MPU6050 nao encontrada em 0x68 ou 0x69. Verifique SDA, SCL e alimentacao.");
        } else {
            Vec3 bias;
            Vec3 initial_accel;
            if (calibrate(address, &bias, &initial_accel)) {
                puts("accelx,accely,accelz,gyrox,gyroy,gyroz,angx,angy,angz");
                stream_samples(address, bias, initial_accel);
            }
            puts("# Falha na leitura I2C. Reiniciando o sensor.");
        }
        sleep_ms(1000);
    }
}
