#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum { MOTOR_1, MOTOR_3, MOTOR_COUNT };
enum { LIMIT_MIN = 1u, LIMIT_MAX = 2u, LIMIT_BOTH = LIMIT_MIN | LIMIT_MAX };
#define ENCODER_PERIOD_MS 5u
// The bench used 640 counts/output revolution (both A edges, x2).
// The PIO decoder counts A and B transitions (x4).
#define ENCODER_COUNTS_PER_OUTPUT_REV 1280.0f
#define ENCODER_DEGREES_PER_COUNT (360.0f / ENCODER_COUNTS_PER_OUTPUT_REV)
#define LOCAL_GRAVITY_MPS2 9.78627f

typedef struct {
    int64_t min_count, max_count;
    unsigned captured;
} TravelCalibration;

void motors_init(void);
void motors_disable_outputs(void);
void motor_task(void *parameter);
void motor_encoder_sample(unsigned motor, int64_t count, uint64_t sampled_at);
void motor_imu_sample(float ax, float ay, float az, float gyro_norm, uint64_t sampled_at);
bool imu_capture_reference(uint64_t now);
bool imu_set_reference(float x, float y, float z);
void imu_zero_heading(void);
void imu_send_reference(void);
unsigned motor_limits(void);
TravelCalibration motor_travel(unsigned motor);
void motor_command(char *line);
void motor_link_down(void);
void motor_send_updates(void);

#endif
