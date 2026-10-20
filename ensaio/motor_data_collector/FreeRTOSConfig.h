#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include <stdint.h>

/* Este projeto usa os dois núcleos do RP2040; o core 0 mantém o tick do RTOS. */
#define configNUMBER_OF_CORES                       2
#define configTICK_CORE                              0

/* Escalonamento preemptivo; ambas as tarefas de aplicação usam prioridade igual. */
#define configUSE_PREEMPTION                         1
#define configUSE_TIME_SLICING                       1
#define configRUN_MULTIPLE_PRIORITIES                1

/* Permite fixar tarefas em um núcleo específico usando afinidade. */
#define configUSE_CORE_AFFINITY                      1

/* O core secundário não executa um hook de idle definido pela aplicação. */
#define configUSE_PASSIVE_IDLE_HOOK                  0
#define configUSE_PORT_OPTIMISED_TASK_SELECTION      0
#define configUSE_TICKLESS_IDLE                      0

/* 100 ticks por segundo: cada tick representa 10 ms. */
#define configTICK_RATE_HZ                           1000
#define configMAX_PRIORITIES                         5
#define configMINIMAL_STACK_SIZE                     256
#define configMAX_TASK_NAME_LEN                      16
#define configTICK_TYPE_WIDTH_IN_BITS                TICK_TYPE_WIDTH_32_BITS

/* Tarefas e objetos usam memória dinâmica, fornecida pelo heap 4 do FreeRTOS. */
#define configSUPPORT_STATIC_ALLOCATION              0
#define configSUPPORT_DYNAMIC_ALLOCATION             1
#define configTOTAL_HEAP_SIZE                        (32 * 1024)
#define configAPPLICATION_ALLOCATED_HEAP             0

/* Habilita primitivas de sincronização e o serviço de timers usado pelo port Pico. */
#define configUSE_MUTEXES                             1
#define configUSE_COUNTING_SEMAPHORES                 1
#define configUSE_TIMERS                              1
#define configTIMER_TASK_PRIORITY                     (configMAX_PRIORITIES - 1)
#define configTIMER_TASK_STACK_DEPTH                  configMINIMAL_STACK_SIZE
#define configTIMER_QUEUE_LENGTH                      10
/* Mantém a tarefa interna de timers no core 0, junto da tarefa de console. */
#define configTIMER_SERVICE_TASK_CORE_AFFINITY        (1u << 0)
#define configUSE_EVENT_GROUPS                       1
#define configUSE_STREAM_BUFFERS                     1

#define configUSE_IDLE_HOOK                          0
#define configUSE_TICK_HOOK                          0
#define configUSE_MALLOC_FAILED_HOOK                 0
#define configCHECK_FOR_STACK_OVERFLOW               0

/* APIs necessárias pelas tarefas e pela integração SMP do Pico SDK. */
#define INCLUDE_vTaskDelay                           1
#define INCLUDE_xTaskDelayUntil                      1
#define INCLUDE_vTaskSuspend                         1
#define INCLUDE_vTaskDelete                          1
#define INCLUDE_xTimerPendFunctionCall               1

#endif /* FREERTOS_CONFIG_H */
