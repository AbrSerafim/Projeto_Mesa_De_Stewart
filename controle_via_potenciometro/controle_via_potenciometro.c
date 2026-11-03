#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* -------------------------------------------------------------------------- */
/* Configuracao de hardware, temporizacao e controlador.                       */
/* -------------------------------------------------------------------------- */

/* Cada bit habilita um nucleo: bit 0 = core 0 e bit 1 = core 1. */
#define CORE_0_AFFINITY_MASK ((UBaseType_t)(1u << 0))
#define CORE_1_AFFINITY_MASK ((UBaseType_t)(1u << 1))

/* Potenciometro: entrada analogica ADC0. */
#define POT_ADC_GPIO 26u
#define POT_ADC_CHANNEL 0u

/* Ponte H: um PWM em IN1 ou IN2, dependendo do sentido. */
#define MOTOR_IN1_GPIO 2u
#define MOTOR_IN2_GPIO 3u

/* Encoder incremental: ambas as bordas do canal A (decodificacao x2). */
#define ENCODER_A_GPIO 6u
#define ENCODER_B_GPIO 7u
#define ENCODER_COUNTS_PER_OUTPUT_REV 640.0f

/* No ensaio: PWM positivo provocava contagem negativa. */
#define ENCODER_SIGN (-1.0f)

/* ADC 0..4095 corresponde a 0..360 graus relativos ao inicio. */
#define POSITION_MIN_DEG 0.0f
#define POSITION_MAX_DEG 360.0f

/* PWM ate 100% para explorar a maxima atuacao disponivel. */
/* Reduza este limite no primeiro teste caso haja risco mecanico. */
#define MOTOR_MAX_DUTY_PERCENT 100.0f

/* Taxas iguais as utilizadas na identificacao do motor. */
#define CONTROL_SAMPLE_PERIOD_MS 20u
#define POT_SAMPLE_PERIOD_MS 20u
#define TELEMETRY_PERIOD_MS 50u

/* Sintonia experimental MAIS AMORTECIDA; validar antes de acoplar a mesa. */
/* Os ganhos anteriores (49.04 / 80.89 / 6.03) causavam ultrapassagem. */
#define PID_KP 185.0f
#define PID_KI 8.0f
#define PID_KD 14.0f
#define PID_TF 0.00010f

/* A referencia segue o potenciometro com velocidade maxima configuravel. */
#define SETPOINT_SLEW_DEG_PER_S 1200.0f

/* A integral atua apenas perto do alvo, evitando acumulacao nos saltos. */
#define INTEGRAL_ENABLE_ERROR_DEG 12.0f
#define INTEGRAL_MAX_PERCENT 15.0f

/* Compensacao simples do atrito; so atua fora da zona de acomodacao. */
#define FRICTION_COMP_ENABLE 1
#define FRICTION_PWM_PERCENT 13.0f
#define FRICTION_MIN_ERROR_DEG 2.0f
#define FRICTION_MAX_SPEED_RPM 15.0f

/* Respeita a resolucao do encoder (0.5625 grau/contagem). */
#define POSITION_TOLERANCE_DEG 0.60f
#define STOP_SPEED_RPM 5.0f

/* PWM proximo de 20 kHz se clk_sys = 125 MHz. */
#define MOTOR_PWM_WRAP 6249u
#define MOTOR_PWM_CLOCK_DIV 1.0f

/* -------------------------------------------------------------------------- */
/* Tipos e objetos de comunicacao entre as tarefas.                            */
/* -------------------------------------------------------------------------- */

typedef struct {
    float reference_deg;
    float position_deg;
    float velocity_rpm;
    float duty_percent;
    float target_deg;
} telemetry_snapshot_t;

/* Filas de um elemento guardam sempre a leitura mais recente. */
static QueueHandle_t g_setpoint_queue;
static QueueHandle_t g_telemetry_queue;

/* ISR e controle executam no core 0. A leitura de int32_t e atomica no RP2040. */
static volatile int32_t g_encoder_count = 0;

/* -------------------------------------------------------------------------- */
/* Saida PWM bidirecional e acionamento do motor.                               */
/* -------------------------------------------------------------------------- */

static void motor_set_signed_duty_percent(float duty_percent)
{
    if (duty_percent > MOTOR_MAX_DUTY_PERCENT) {
        duty_percent = MOTOR_MAX_DUTY_PERCENT;
    } else if (duty_percent < -MOTOR_MAX_DUTY_PERCENT) {
        duty_percent = -MOTOR_MAX_DUTY_PERCENT;
    }

    const uint16_t level = (uint16_t)(
        (fabsf(duty_percent) * (float)(MOTOR_PWM_WRAP + 1u)) / 100.0f);

    /* Nunca comanda os dois sentidos simultaneamente. */
    pwm_set_gpio_level(MOTOR_IN1_GPIO, 0u);
    pwm_set_gpio_level(MOTOR_IN2_GPIO, 0u);

    if (duty_percent > 0.0f) {
        pwm_set_gpio_level(MOTOR_IN1_GPIO, level);
    } else if (duty_percent < 0.0f) {
        pwm_set_gpio_level(MOTOR_IN2_GPIO, level);
    }
}

static void motor_outputs_init(void)
{
    gpio_set_function(MOTOR_IN1_GPIO, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR_IN2_GPIO, GPIO_FUNC_PWM);

    /* GPIO 2 e 3 compartilham o mesmo slice, em canais distintos. */
    const uint slice = pwm_gpio_to_slice_num(MOTOR_IN1_GPIO);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_wrap(&config, MOTOR_PWM_WRAP);
    pwm_config_set_clkdiv(&config, MOTOR_PWM_CLOCK_DIV);

    pwm_init(slice, &config, false);
    pwm_set_gpio_level(MOTOR_IN1_GPIO, 0u);
    pwm_set_gpio_level(MOTOR_IN2_GPIO, 0u);
    pwm_set_enabled(slice, true);
}

/* -------------------------------------------------------------------------- */
/* Leitura do encoder e posicao angular acumulada.                              */
/* -------------------------------------------------------------------------- */

static void encoder_gpio_callback(uint gpio, uint32_t events)
{
    if ((gpio != ENCODER_A_GPIO) ||
        ((events & (GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL)) == 0u)) {
        return;
    }

    const bool phase_a = gpio_get(ENCODER_A_GPIO);
    const bool phase_b = gpio_get(ENCODER_B_GPIO);

    if (phase_a == phase_b) {
        ++g_encoder_count;
    } else {
        --g_encoder_count;
    }
}

static void encoder_inputs_init(void)
{
    gpio_init(ENCODER_A_GPIO);
    gpio_set_dir(ENCODER_A_GPIO, GPIO_IN);
    gpio_pull_up(ENCODER_A_GPIO);

    gpio_init(ENCODER_B_GPIO);
    gpio_set_dir(ENCODER_B_GPIO, GPIO_IN);
    gpio_pull_up(ENCODER_B_GPIO);

    gpio_set_irq_enabled_with_callback(
        ENCODER_A_GPIO,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true,
        encoder_gpio_callback);
}

/* -------------------------------------------------------------------------- */
/* Tarefa de aquisicao do setpoint angular.                                     */
/* -------------------------------------------------------------------------- */

static void potentiometer_task(void *parameters)
{
    (void)parameters;

    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        const uint16_t adc_raw = adc_read();
        (void)xQueueOverwrite(g_setpoint_queue, &adc_raw);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(POT_SAMPLE_PERIOD_MS));
    }
}

/* -------------------------------------------------------------------------- */
/* Tarefa de controle PIDF de posicao angular.                                  */
/* -------------------------------------------------------------------------- */

static void position_control_task(void *parameters)
{
    (void)parameters;

    uint16_t adc_raw = 0u;
    TickType_t last_wake = xTaskGetTickCount();
    uint64_t previous_us = time_us_64();
    int32_t previous_count = g_encoder_count;

    float integral_percent = 0.0f;
    float filtered_speed_rad_s = 0.0f;
    float shaped_reference_deg = 0.0f;
    bool reference_initialized = false;

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_SAMPLE_PERIOD_MS));

        const uint64_t now_us = time_us_64();
        float dt = (float)(now_us - previous_us) / 1000000.0f;
        previous_us = now_us;
        if (dt <= 0.0f) {
            dt = (float)CONTROL_SAMPLE_PERIOD_MS / 1000.0f;
        }

        (void)xQueuePeek(g_setpoint_queue, &adc_raw, 0u);

        const float target_deg = POSITION_MIN_DEG +
            ((float)adc_raw / 4095.0f) * (POSITION_MAX_DEG - POSITION_MIN_DEG);

        const int32_t count = g_encoder_count;
        const int32_t delta_count = count - previous_count;
        previous_count = count;

        /* Posicao relativa ao instante de ligamento, com sentido corrigido. */
        const float position_rad = ENCODER_SIGN *
            ((float)count * (2.0f * 3.14159265359f)) /
            ENCODER_COUNTS_PER_OUTPUT_REV;
        const float position_deg = position_rad * (180.0f / 3.14159265359f);

        /* Evita um salto de referencia artificial na primeira amostra. */
        if (!reference_initialized) {
            shaped_reference_deg = position_deg;
            reference_initialized = true;
        }

        /* Limita a rapidez da referencia, nao a velocidade fisica do motor. */
        const float max_reference_change = SETPOINT_SLEW_DEG_PER_S * dt;
        float reference_change = target_deg - shaped_reference_deg;
        if (reference_change > max_reference_change) {
            reference_change = max_reference_change;
        } else if (reference_change < -max_reference_change) {
            reference_change = -max_reference_change;
        }
        shaped_reference_deg += reference_change;

        const float error_deg = shaped_reference_deg - position_deg;
        const float error_rad = error_deg * (3.14159265359f / 180.0f);

        /* Derivada da posicao medida: sem chute derivativo no setpoint. */
        const float speed_rad_s = ENCODER_SIGN *
            ((float)delta_count * (2.0f * 3.14159265359f)) /
            (ENCODER_COUNTS_PER_OUTPUT_REV * dt);
        const float speed_rpm = speed_rad_s * (60.0f / (2.0f * 3.14159265359f));

        const float alpha = PID_TF / (PID_TF + dt);
        filtered_speed_rad_s = alpha * filtered_speed_rad_s +
            (1.0f - alpha) * speed_rad_s;

        /* Nao integra durante grandes deslocamentos e reversoes bruscas. */
        if (fabsf(error_deg) > INTEGRAL_ENABLE_ERROR_DEG) {
            integral_percent = 0.0f;
        } else {
            const float pre = PID_KP * error_rad + integral_percent -
                PID_KD * filtered_speed_rad_s;
            const bool can_integrate =
                ((pre >= -MOTOR_MAX_DUTY_PERCENT) &&
                 (pre <= MOTOR_MAX_DUTY_PERCENT)) ||
                ((pre > MOTOR_MAX_DUTY_PERCENT) && (error_rad < 0.0f)) ||
                ((pre < -MOTOR_MAX_DUTY_PERCENT) && (error_rad > 0.0f));

            if (can_integrate) {
                integral_percent += PID_KI * error_rad * dt;
            }
            if (integral_percent > INTEGRAL_MAX_PERCENT) {
                integral_percent = INTEGRAL_MAX_PERCENT;
            } else if (integral_percent < -INTEGRAL_MAX_PERCENT) {
                integral_percent = -INTEGRAL_MAX_PERCENT;
            }
        }

        float duty_percent = PID_KP * error_rad + integral_percent -
            PID_KD * filtered_speed_rad_s;

        /* Compensacao minima de atrito somente se quase parado e longe do alvo. */
        if (FRICTION_COMP_ENABLE &&
            (fabsf(error_deg) > FRICTION_MIN_ERROR_DEG) &&
            (fabsf(speed_rpm) < FRICTION_MAX_SPEED_RPM) &&
            (fabsf(duty_percent) < FRICTION_PWM_PERCENT)) {
            duty_percent = (error_deg > 0.0f) ?
                FRICTION_PWM_PERCENT : -FRICTION_PWM_PERCENT;
        }

        /* Na vizinhanca do alvo, para de caçar contagens do encoder. */
        if ((fabsf(error_deg) <= POSITION_TOLERANCE_DEG) &&
            (fabsf(speed_rpm) <= STOP_SPEED_RPM) &&
            (fabsf(target_deg - shaped_reference_deg) <= POSITION_TOLERANCE_DEG)) {
            duty_percent = 0.0f;
            integral_percent = 0.0f;
        }

        if (duty_percent > MOTOR_MAX_DUTY_PERCENT) {
            duty_percent = MOTOR_MAX_DUTY_PERCENT;
        } else if (duty_percent < -MOTOR_MAX_DUTY_PERCENT) {
            duty_percent = -MOTOR_MAX_DUTY_PERCENT;
        }

        motor_set_signed_duty_percent(duty_percent);

        const telemetry_snapshot_t snapshot = {
            .reference_deg = shaped_reference_deg,
            .position_deg = position_deg,
            .velocity_rpm = speed_rpm,
            .duty_percent = duty_percent,
            .target_deg = target_deg
        };
        (void)xQueueOverwrite(g_telemetry_queue, &snapshot);
    }
}

/* -------------------------------------------------------------------------- */
/* Envio dos dados ao Muino Serial Plotter.                                     */
/* -------------------------------------------------------------------------- */

/* Formata numero com sinal e duas casas, sem depender de printf float. */
static void format_fixed_2(char *output, size_t output_size, float value)
{
    const int32_t scaled = (int32_t)(fabsf(value) * 100.0f + 0.5f);
    snprintf(output, output_size, "%s%ld.%02ld",
        (value < 0.0f) ? "-" : "",
        (long)(scaled / 100),
        (long)(scaled % 100));
}

static void telemetry_task(void *parameters)
{
    (void)parameters;

    telemetry_snapshot_t snapshot = {0};
    TickType_t last_wake = xTaskGetTickCount();

    char reference[24];
    char position[24];
    char velocity[24];
    char duty[24];
    char target[24];

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));

        if (xQueuePeek(g_telemetry_queue, &snapshot, 0u) == pdPASS) {
            format_fixed_2(reference, sizeof(reference), snapshot.reference_deg);
            format_fixed_2(position, sizeof(position), snapshot.position_deg);
            format_fixed_2(velocity, sizeof(velocity), snapshot.velocity_rpm);
            format_fixed_2(duty, sizeof(duty), snapshot.duty_percent);
            format_fixed_2(target, sizeof(target), snapshot.target_deg);

            printf("target:%s,setpoint:%s,position:%s,velocity:%s,pwm:%s\n",
                target, reference, position, velocity, duty);
        }
    }
}

/* -------------------------------------------------------------------------- */
/* Tarefa de sinalizacao visual.                                               */
/* -------------------------------------------------------------------------- */

static void led_task(void *parameters)
{
    (void)parameters;

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    for (;;) {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(500u));
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(500u));
    }
}

/* -------------------------------------------------------------------------- */
/* Inicializacao do firmware e criacao das tarefas.                             */
/* -------------------------------------------------------------------------- */

int main(void)
{
    stdio_init_all();

    motor_outputs_init();

    adc_init();
    adc_gpio_init(POT_ADC_GPIO);
    adc_select_input(POT_ADC_CHANNEL);

    g_setpoint_queue = xQueueCreate(1u, sizeof(uint16_t));
    g_telemetry_queue = xQueueCreate(1u, sizeof(telemetry_snapshot_t));

    if ((g_setpoint_queue == NULL) || (g_telemetry_queue == NULL)) {
        for (;;) {
            tight_loop_contents();
        }
    }

    /* Encoder e interrupcao permanecem no core 0, como no exemplo original. */
    encoder_inputs_init();

    if (xTaskCreateAffinitySet(potentiometer_task,
                               "pot_adc", 256u, NULL,
                               tskIDLE_PRIORITY + 2u,
                               CORE_0_AFFINITY_MASK, NULL) != pdPASS ||
        xTaskCreateAffinitySet(position_control_task,
                               "position_pidf", 512u, NULL,
                               tskIDLE_PRIORITY + 3u,
                               CORE_0_AFFINITY_MASK, NULL) != pdPASS ||
        xTaskCreateAffinitySet(telemetry_task,
                               "telemetry", 512u, NULL,
                               tskIDLE_PRIORITY + 1u,
                               CORE_0_AFFINITY_MASK, NULL) != pdPASS ||
        xTaskCreateAffinitySet(led_task,
                               "led", 256u, NULL,
                               tskIDLE_PRIORITY + 1u,
                               CORE_1_AFFINITY_MASK, NULL) != pdPASS) {
        for (;;) {
            tight_loop_contents();
        }
    }

    vTaskStartScheduler();

    for (;;) {
        tight_loop_contents();
    }
}
