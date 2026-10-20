#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

#include "FreeRTOS.h"
#include "task.h"

/* -------------------------------------------------------------------------- */
/* Configuracao do ensaio, hardware e temporizacao.                            */
/* -------------------------------------------------------------------------- */

/* Cada bit habilita um nucleo: bit 0 = core 0 e bit 1 = core 1. */
#define CORE_0_AFFINITY_MASK ((UBaseType_t)(1u << 0))
#define CORE_1_AFFINITY_MASK ((UBaseType_t)(1u << 1))

/* Ponte H: dois pinos PWM para permitir movimento nos dois sentidos.
 * IMPORTANTE: confirme a tabela de verdade do driver antes de energizar. */
#define MOTOR_IN1_GPIO 2u
#define MOTOR_IN2_GPIO 3u

/* Encoder incremental: duas bordas de A; B fornece o sentido (x2). */
#define ENCODER_A_GPIO 6u
#define ENCODER_B_GPIO 7u
#define ENCODER_COUNTS_PER_OUTPUT_REV 640.0f
#define ENCODER_SIGN 1                         /* Troque para -1 se necessario. */

/* Ajuste o sinal ao limite seguro do conjunto motor + fonte + carga. */
#define PWM_LIMIT_PERCENT 100.0f
#define MOTOR_PWM_WRAP 6249u
#define MOTOR_PWM_CLOCK_DIV 1.0f

/* Amostragem */
#define SAMPLE_PERIOD_MS 2u //<-------------
#define MAX_SAMPLES 4000u
#define START_DELAY_MS 2000u
#define DUMP_DELAY_MS 1000u

/* Selecao do sinal: altere apenas esta constante para escolher o ensaio. */
#define SIGNAL_STEP_REVERSAL 1u
#define SIGNAL_SINE 2u
#define SIGNAL_PRBS 3u
#define SIGNAL_MULTISINE 4u
#define SIGNAL_CHIRP 5u
#define SIGNAL_FILTERED_RANDOM 6u
#define SIGNAL_RANDOM_MULTILEVEL 7u
#define SIGNAL_BIPOLAR_MULTILEVEL 8u
#define EXPERIMENT_SIGNAL SIGNAL_BIPOLAR_MULTILEVEL

/* Modo 0 = coleta em RAM e imprime ao final (recomendado).
 * Modo 1 = imprime enquanto ensaia; pode prejudicar o determinismo. */
#define OUTPUT_BUFFERED_CSV 0u
#define OUTPUT_LIVE_MUINO 1u
#define OUTPUT_MODE OUTPUT_BUFFERED_CSV

/* Perfil temporal; todas as frequencias em Hz. */
#define EXPERIMENT_DURATION_MS 30000u
#define SIGNAL_AMPLITUDE_PERCENT 35.0f
#define SIGNAL_BIAS_PERCENT 0.0f
#define STEP_FORWARD_START_S 2.0f
#define STEP_REVERSE_START_S 12.0f
#define STEP_STOP_S 22.0f
#define SINE_FREQUENCY_HZ 0.40f
#define PRBS_BIT_DURATION_MS 300u
#define RANDOM_LEVEL_DURATION_MS 500u
#define RANDOM_LEVEL_STEP_PERCENT 10u

/* PRMS bipolar: amplitude de 10 a 100%, com mudancas de sinal frequentes. */
#define BIPOLAR_LEVEL_DURATION_MS 400u
#define BIPOLAR_LEVEL_STEP_PERCENT 10u
#define BIPOLAR_MAX_SAME_DIRECTION 3u

/* A ponte H permanece sem PWM antes de inverter o sentido. */
#define MOTOR_REVERSAL_DEADTIME_MS 100u
#define CHIRP_START_HZ 0.05f
#define CHIRP_END_HZ 3.0f
#define FILTERED_RANDOM_ALPHA 0.08f

#define TWO_PI 6.2831853071795864769f

/* -------------------------------------------------------------------------- */
/* Amostras armazenadas em RAM e estados compartilhados entre os nucleos.      */
/* -------------------------------------------------------------------------- */

/* 12 bytes/amostra no RP2040: ~48 kB para 4000 pontos. */
typedef struct {
    uint32_t time_us;
    int32_t encoder_count;
    int16_t pwm_tenths;
    int16_t reserved;
} experiment_sample_t;

static experiment_sample_t g_samples[MAX_SAMPLES];
static uint32_t g_sample_count = 0u;
static uint32_t g_late_samples = 0u;
static uint32_t g_max_timing_error_us = 0u;

/* __atomic_* evita leituras rasgadas ou corridas entre cores. */
static int32_t g_encoder_position = 0;
static uint32_t g_encoder_ready = 0u;
static uint32_t g_experiment_finished = 0u;
static uint32_t g_encoder_edges = 0u;

/* -------------------------------------------------------------------------- */
/* PWM bidirecional.                                                          */
/* -------------------------------------------------------------------------- */

static float clamp_percent(float value)
{
    if (value > PWM_LIMIT_PERCENT) {
        return PWM_LIMIT_PERCENT;
    }
    if (value < -PWM_LIMIT_PERCENT) {
        return -PWM_LIMIT_PERCENT;
    }
    return value;
}

/* PWM positivo: IN1; negativo: IN2; zero: ambos desligados (coast/drive-off
 * depende da ponte H). Verifique o datasheet antes de habilitar reversao. */
static void motor_set_signed_duty_percent(float duty_percent)
{
    duty_percent = clamp_percent(duty_percent);

    const float magnitude = fabsf(duty_percent);
    uint16_t level = (uint16_t)((magnitude *
        (float)(MOTOR_PWM_WRAP + 1u)) / 100.0f);

    if (level > MOTOR_PWM_WRAP) {
        level = MOTOR_PWM_WRAP;
    }

    if (duty_percent > 0.0f) {
        pwm_set_gpio_level(MOTOR_IN2_GPIO, 0u);
        pwm_set_gpio_level(MOTOR_IN1_GPIO, level);
    } else if (duty_percent < 0.0f) {
        pwm_set_gpio_level(MOTOR_IN1_GPIO, 0u);
        pwm_set_gpio_level(MOTOR_IN2_GPIO, level);
    } else {
        pwm_set_gpio_level(MOTOR_IN1_GPIO, 0u);
        pwm_set_gpio_level(MOTOR_IN2_GPIO, 0u);
    }
}

static void motor_outputs_init(void)
{
    gpio_set_function(MOTOR_IN1_GPIO, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR_IN2_GPIO, GPIO_FUNC_PWM);

    const uint slice_1 = pwm_gpio_to_slice_num(MOTOR_IN1_GPIO);
    const uint slice_2 = pwm_gpio_to_slice_num(MOTOR_IN2_GPIO);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_wrap(&config, MOTOR_PWM_WRAP);
    pwm_config_set_clkdiv(&config, MOTOR_PWM_CLOCK_DIV);

    pwm_init(slice_1, &config, false);
    if (slice_2 != slice_1) {
        pwm_init(slice_2, &config, false);
    }

    motor_set_signed_duty_percent(0.0f);
    pwm_set_enabled(slice_1, true);
    if (slice_2 != slice_1) {
        pwm_set_enabled(slice_2, true);
    }
}

/* -------------------------------------------------------------------------- */
/* Encoder: interrupcao e tarefa presas ao core 1.                             */
/* -------------------------------------------------------------------------- */

static void encoder_gpio_callback(uint gpio, uint32_t events)
{
    if ((gpio != ENCODER_A_GPIO) ||
        ((events & (GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL)) == 0u)) {
        return;
    }

    const bool phase_a = gpio_get(ENCODER_A_GPIO);
    const bool phase_b = gpio_get(ENCODER_B_GPIO);
    const int32_t direction = ((phase_a == phase_b) ? 1 : -1) * ENCODER_SIGN;

    __atomic_fetch_add(&g_encoder_position, direction, __ATOMIC_RELAXED);
    __atomic_fetch_add(&g_encoder_edges, 1u, __ATOMIC_RELAXED);
}

static void encoder_task(void *parameters)
{
    (void)parameters;

    gpio_init(ENCODER_A_GPIO);
    gpio_set_dir(ENCODER_A_GPIO, GPIO_IN);
    gpio_pull_up(ENCODER_A_GPIO);

    gpio_init(ENCODER_B_GPIO);
    gpio_set_dir(ENCODER_B_GPIO, GPIO_IN);
    gpio_pull_up(ENCODER_B_GPIO);

    /* A API registra a callback no nucleo que a executa: core 1. */
    gpio_set_irq_enabled_with_callback(
        ENCODER_A_GPIO,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true,
        encoder_gpio_callback);

    __atomic_store_n(&g_encoder_ready, 1u, __ATOMIC_RELEASE);

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000u));
    }
}

/* -------------------------------------------------------------------------- */
/* Geracao deterministica das entradas de identificacao.                      */
/* -------------------------------------------------------------------------- */

static uint16_t g_lfsr = 0xACE1u;
static int32_t g_prbs_bit_index = -1;
static float g_prbs_value = 1.0f;
static float g_random_filtered = 0.0f;
static int32_t g_random_sample_index = -1;
static int32_t g_multilevel_index = -1;
static uint8_t g_multilevel_order[11];
static float g_multilevel_percent = 0.0f;
static int32_t g_bipolar_index = -1;
static float g_bipolar_percent = 0.0f;
static int32_t g_bipolar_sign = 1;
static uint32_t g_bipolar_same_direction = 0u;
static uint32_t g_reverse_wait_until_ms = 0u;
static int32_t g_last_motor_sign = 0;
static bool g_reverse_waiting = false;

/* PRBS de 16 bits; avanca apenas quando chega a hora de trocar de bit. */
static uint16_t lfsr_next(void)
{
    const uint16_t bit = (uint16_t)(
        ((g_lfsr >> 0) ^ (g_lfsr >> 2) ^
         (g_lfsr >> 3) ^ (g_lfsr >> 5)) & 1u);

    g_lfsr = (uint16_t)((g_lfsr >> 1) | (bit << 15));
    return g_lfsr;
}

/* Em cada bloco de onze intervalos, visita todos os niveis de 0 a 100%. */
static void multilevel_shuffle(void)
{
    for (uint32_t i = 0u; i < 11u; ++i) {
        g_multilevel_order[i] = (uint8_t)i;
    }

    for (int32_t i = 10; i > 0; --i) {
        const uint32_t j = (uint32_t)lfsr_next() % ((uint32_t)i + 1u);
        const uint8_t temporary = g_multilevel_order[i];
        g_multilevel_order[i] = g_multilevel_order[j];
        g_multilevel_order[j] = temporary;
    }
}

/* Retorna sinal em {-1, 0, +1}. */
static int32_t duty_sign(float duty)
{
    return (duty > 0.0f) ? 1 : ((duty < 0.0f) ? -1 : 0);
}

/* Garante PWM zero por um intervalo quando o sentido solicitado inverte.
 * Esta funcao nao bloqueia a tarefa e o CSV registra o PWM efetivo. */
static float reversal_interlock(float requested_duty, uint32_t elapsed_ms)
{
    const int32_t requested_sign = duty_sign(requested_duty);

    if (g_reverse_waiting) {
        if (elapsed_ms < g_reverse_wait_until_ms) {
            return 0.0f;
        }

        g_reverse_waiting = false;
        g_last_motor_sign = 0;
    }

    if ((requested_sign != 0) && (g_last_motor_sign != 0) &&
        (requested_sign != g_last_motor_sign)) {
        g_reverse_waiting = true;
        g_reverse_wait_until_ms = elapsed_ms + MOTOR_REVERSAL_DEADTIME_MS;
        return 0.0f;
    }

    /* Uma fase em zero nao apaga o sentido anterior fora da reversao. */
    if (requested_sign != 0) {
        g_last_motor_sign = requested_sign;
    }

    return requested_duty;
}

static float signal_generate(uint32_t elapsed_ms)
{
    const float t = (float)elapsed_ms / 1000.0f;
    const float duration_s = (float)EXPERIMENT_DURATION_MS / 1000.0f;
    float normalized = 0.0f;

#if EXPERIMENT_SIGNAL == SIGNAL_STEP_REVERSAL
    if (t >= STEP_FORWARD_START_S && t < STEP_REVERSE_START_S) {
        normalized = 1.0f;
    } else if (t >= STEP_REVERSE_START_S && t < STEP_STOP_S) {
        normalized = -1.0f;
    }
#elif EXPERIMENT_SIGNAL == SIGNAL_SINE
    normalized = sinf(TWO_PI * SINE_FREQUENCY_HZ * t);
#elif EXPERIMENT_SIGNAL == SIGNAL_PRBS
    const int32_t bit_index = (int32_t)(elapsed_ms / PRBS_BIT_DURATION_MS);
    while (g_prbs_bit_index < bit_index) {
        g_prbs_value = ((lfsr_next() & 1u) != 0u) ? 1.0f : -1.0f;
        ++g_prbs_bit_index;
    }
    /* PRBS binario unidirecional: alterna exclusivamente entre 0 e 100%. */
    return clamp_percent(50.0f + 50.0f * g_prbs_value);
#elif EXPERIMENT_SIGNAL == SIGNAL_RANDOM_MULTILEVEL
    /* PRMS unidirecional: onze niveis (0,10,...,100). */
    const int32_t index = (int32_t)(elapsed_ms / RANDOM_LEVEL_DURATION_MS);
    while (g_multilevel_index < index) {
        ++g_multilevel_index;
        if ((g_multilevel_index % 11) == 0) {
            multilevel_shuffle();
        }
        g_multilevel_percent = (float)(
            g_multilevel_order[g_multilevel_index % 11] *
            RANDOM_LEVEL_STEP_PERCENT);
    }
    return clamp_percent(g_multilevel_percent);
#elif EXPERIMENT_SIGNAL == SIGNAL_BIPOLAR_MULTILEVEL
    /* Niveis assinados de -100 a -10 e +10 a +100%, com inversoes
     * aleatorias e forçadas depois de algumas permanencias no sentido. */
    const int32_t index = (int32_t)(elapsed_ms / BIPOLAR_LEVEL_DURATION_MS);
    while (g_bipolar_index < index) {
        ++g_bipolar_index;

        const bool must_reverse =
            (g_bipolar_same_direction >= BIPOLAR_MAX_SAME_DIRECTION);
        const bool random_reverse = ((lfsr_next() & 1u) != 0u);
        if (must_reverse || random_reverse) {
            g_bipolar_sign = -g_bipolar_sign;
            g_bipolar_same_direction = 1u;
        } else {
            ++g_bipolar_same_direction;
        }

        const uint32_t level = 1u + (lfsr_next() %
            (100u / BIPOLAR_LEVEL_STEP_PERCENT));
        g_bipolar_percent = (float)g_bipolar_sign * (float)level *
            (float)BIPOLAR_LEVEL_STEP_PERCENT;
    }
    return clamp_percent(g_bipolar_percent);
#elif EXPERIMENT_SIGNAL == SIGNAL_MULTISINE
    /* Frequencias distintas e nao harmonicas; normalizado conservadoramente. */
    normalized = (
        sinf(TWO_PI * 0.13f * t) +
        sinf(TWO_PI * 0.37f * t + 0.7f) +
        sinf(TWO_PI * 0.91f * t + 1.9f) +
        sinf(TWO_PI * 1.73f * t + 2.4f)) / 4.0f;
#elif EXPERIMENT_SIGNAL == SIGNAL_CHIRP
    const float slope = (CHIRP_END_HZ - CHIRP_START_HZ) / duration_s;
    normalized = sinf(TWO_PI * (CHIRP_START_HZ * t + 0.5f * slope * t * t));
#elif EXPERIMENT_SIGNAL == SIGNAL_FILTERED_RANDOM
    const int32_t index = (int32_t)(elapsed_ms / SAMPLE_PERIOD_MS);
    while (g_random_sample_index < index) {
        const float white = ((float)(lfsr_next() & 0xFFu) / 127.5f) - 1.0f;
        g_random_filtered += FILTERED_RANDOM_ALPHA *
            (white - g_random_filtered);
        ++g_random_sample_index;
    }
    normalized = g_random_filtered;
#else
#error "EXPERIMENT_SIGNAL invalido"
#endif

    return clamp_percent(SIGNAL_BIAS_PERCENT +
                         SIGNAL_AMPLITUDE_PERCENT * normalized);
}

/* -------------------------------------------------------------------------- */
/* Aquisicao (core 0): aplica entrada, amostra posicao, registra na RAM.        */
/* -------------------------------------------------------------------------- */

static void experiment_task(void *parameters)
{
    (void)parameters;

    while (__atomic_load_n(&g_encoder_ready, __ATOMIC_ACQUIRE) == 0u) {
        vTaskDelay(pdMS_TO_TICKS(1u));
    }

    motor_set_signed_duty_percent(0.0f);
    vTaskDelay(pdMS_TO_TICKS(START_DELAY_MS));

    /* Posicao relativa: zerada antes do inicio do experimento. */
    __atomic_store_n(&g_encoder_position, 0, __ATOMIC_RELEASE);

    const uint64_t start_us = time_us_64();
    TickType_t last_wake = xTaskGetTickCount();
    const uint32_t requested_samples =
        (EXPERIMENT_DURATION_MS / SAMPLE_PERIOD_MS) + 1u;

    for (uint32_t k = 0u;
         k < requested_samples && k < MAX_SAMPLES;
         ++k) {

        if (k > 0u) {
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
        }

        const uint64_t sample_time_us = time_us_64();
        const uint32_t elapsed_us = (uint32_t)(sample_time_us - start_us);
        const uint32_t expected_us = k * SAMPLE_PERIOD_MS * 1000u;
        const uint32_t timing_error_us = (elapsed_us > expected_us) ?
            (elapsed_us - expected_us) : (expected_us - elapsed_us);

        if (timing_error_us > g_max_timing_error_us) {
            g_max_timing_error_us = timing_error_us;
        }
        if (timing_error_us > (SAMPLE_PERIOD_MS * 1000u / 4u)) {
            ++g_late_samples;
        }

        const int32_t encoder_count =
            __atomic_load_n(&g_encoder_position, __ATOMIC_RELAXED);

        /* No instante k: registra posicao e o novo PWM aplicado. */
        const uint32_t nominal_ms = k * SAMPLE_PERIOD_MS;
        const float requested_duty = signal_generate(nominal_ms);
        const float duty_percent = reversal_interlock(requested_duty, nominal_ms);
        const int16_t pwm_tenths = (int16_t)lroundf(duty_percent * 10.0f);

        g_samples[k].time_us = elapsed_us;
        g_samples[k].encoder_count = encoder_count;
        g_samples[k].pwm_tenths = pwm_tenths;
        g_samples[k].reserved = 0;
        g_sample_count = k + 1u;

        motor_set_signed_duty_percent((float)pwm_tenths / 10.0f);

#if OUTPUT_MODE == OUTPUT_LIVE_MUINO
        /* Visualizacao exploratoria: printf pode atrasar a aquisicao. */
        const float angle_rad = (float)encoder_count *
            TWO_PI / ENCODER_COUNTS_PER_OUTPUT_REV;
        printf("duty:%.2f,angle_rad:%.6f\n",
            (double)duty_percent, (double)angle_rad);
#endif
    }

    motor_set_signed_duty_percent(0.0f);
    __atomic_store_n(&g_experiment_finished, 1u, __ATOMIC_RELEASE);
    vTaskDelete(NULL);
}

/* -------------------------------------------------------------------------- */
/* Telemetria: sem impressao durante o ensaio no modo recomendado.             */
/* -------------------------------------------------------------------------- */

/* Imprime tempo, PWM efetivo, angulo, RPM assinado e contagem de encoder.
 * O sinal aplicado e mantido constante ate a amostra seguinte (ZOH).
 * As linhas de CSV iniciam somente apos o cabecalho DATA_BEGIN. */
static void dump_csv(void)
{
    printf("# DATA_BEGIN\n");
    printf("time_s,duty_percent,angle_rad,velocity_rpm,encoder_count\n");

    for (uint32_t k = 0u; k < g_sample_count; ++k) {
        const experiment_sample_t *sample = &g_samples[k];
        const float time_s = (float)sample->time_us / 1000000.0f;
        const float duty = (float)sample->pwm_tenths / 10.0f;
        const float angle = (float)sample->encoder_count *
            TWO_PI / ENCODER_COUNTS_PER_OUTPUT_REV;

        float rpm = 0.0f;
        if (k > 0u) {
            const int32_t dc = sample->encoder_count -
                g_samples[k - 1u].encoder_count;
            const uint32_t dt_us = sample->time_us -
                g_samples[k - 1u].time_us;

            if (dt_us > 0u) {
                rpm = ((float)dc * 60000000.0f) /
                    (ENCODER_COUNTS_PER_OUTPUT_REV * (float)dt_us);
            }
        }

        printf("%.6f,%.2f,%.6f,%.3f,%ld\n",
            (double)time_s, (double)duty, (double)angle,
            (double)rpm, (long)sample->encoder_count);
    }

    printf("# DATA_END\n");
    printf("# samples=%lu,late_samples=%lu,max_timing_error_us=%lu,encoder_edges=%lu\n",
        (unsigned long)g_sample_count,
        (unsigned long)g_late_samples,
        (unsigned long)g_max_timing_error_us,
        (unsigned long)__atomic_load_n(&g_encoder_edges, __ATOMIC_RELAXED));
    fflush(stdout);
}

static void telemetry_task(void *parameters)
{
    (void)parameters;

    while (__atomic_load_n(&g_experiment_finished, __ATOMIC_ACQUIRE) == 0u) {
        vTaskDelay(pdMS_TO_TICKS(50u));
    }

    vTaskDelay(pdMS_TO_TICKS(DUMP_DELAY_MS));

#if OUTPUT_MODE == OUTPUT_BUFFERED_CSV
    dump_csv();
#else
    printf("# live_run_finished,samples=%lu,late_samples=%lu,max_timing_error_us=%lu\n",
        (unsigned long)g_sample_count,
        (unsigned long)g_late_samples,
        (unsigned long)g_max_timing_error_us);
#endif

    vTaskDelete(NULL);
}

/* -------------------------------------------------------------------------- */
/* Inicializacao do firmware e criacao das tarefas SMP.                        */
/* -------------------------------------------------------------------------- */

int main(void)
{
    stdio_init_all();

    sleep_ms(3000);

    motor_outputs_init();

    if (xTaskCreateAffinitySet(encoder_task,
                               "encoder_core1",
                               256,
                               NULL,
                               tskIDLE_PRIORITY + 3,
                               CORE_1_AFFINITY_MASK,
                               NULL) != pdPASS ||
        xTaskCreateAffinitySet(experiment_task,
                               "experiment_core0",
                               768,
                               NULL,
                               tskIDLE_PRIORITY + 3,
                               CORE_0_AFFINITY_MASK,
                               NULL) != pdPASS ||
        xTaskCreateAffinitySet(telemetry_task,
                               "telemetry_core0",
                               768,
                               NULL,
                               tskIDLE_PRIORITY + 1,
                               CORE_0_AFFINITY_MASK,
                               NULL) != pdPASS) {
        motor_set_signed_duty_percent(0.0f);
        for (;;) {
            tight_loop_contents();
        }
    }

    vTaskStartScheduler();

    motor_set_signed_duty_percent(0.0f);
    for (;;) {
        tight_loop_contents();
    }
}