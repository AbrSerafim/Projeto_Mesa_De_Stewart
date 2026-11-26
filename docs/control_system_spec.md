# Especificação do controle

Câmera → YOLO → posição X/Y em pixels → correções em pontos percentuais → matriz de curso → USB CDC → malha de posição da Pico → malha de velocidade → PWM nos Enables da L298.

## Aquisição e estado

A captura data o quadro na entrega de OpenCV usando o relógio monotônico do PC. A inferência publica um único resultado recente e consome no máximo um quadro pendente. O controle processa timestamps crescentes, em até 20 Hz. O renderizador consome snapshots. A aba sinaliza câmera, YOLO, controle, transmissão e ACK em frequências distintas. A idade usa o timestamp de captura. A duração da inferência mede o trabalho do detector, incluindo ArUco quando habilitado. O tempo de geração mede o trabalho desde a seleção da observação até o cálculo do comando.

Home continua em 40% do curso. Os neutros da bola são independentes, com 40% em cada motor como valor inicial configurado. Conclua o nivelamento com a IMU, capture os neutros na aba Câmera e salve os parâmetros. Na Pico, cada sessão passa por centralização estática, rastreamento ativo ou estado terminal. O PC confirma o vínculo, exige câmera e setpoint recentes, aguarda o estado de centralização e mantém o watchdog de referências com observações novas. ACK na centralização confirma recepção sem mudar a meta. A remoção do setpoint, a troca de classe e a saída da aba interrompem o controle. Após falha, a autocorreção exige rearme explícito com os motores em freio, fim de curso coerente e encoder recente.

## Controladores da bola

O eixo usa polaridade `s = ±1`, erro geométrico `e = r − y` e período `T` entre observações usadas. Unidades de Kp, Ki, Kd e Tf são p.p./px, p.p./(px·s), p.p.·s/px e s. O modo P habilita P, PI habilita P/I, PID usa P/I/D com Tf=0 e PIDF usa Tf configurado. As equações de Euler implícito são:

```text
P[k] = s Kp e[k]
I*[k] = I[k-1] + s Ki T e[k]
D[k] = (Tf D[k-1] − s Kd (y[k] − y[k-1])) / (Tf + T)
u[k] = P[k] + I[k] + D[k]
motor_pct = clamp(neutro + matriz × clamp_eixo(u))
```

A integração condicional bloqueia incrementos que aprofundam a saturação de um eixo ou de qualquer motor. Os valores do perfil são limitados antes da transformação e os limites de curso são aplicados depois dela. A escala e a polaridade se aplicam também à derivada. Mudanças de setpoint não geram salto derivativo. Na primeira amostra, na recuperação e após um intervalo acima de 320 ms o estado é inicializado pela medição atual. Timestamps duplicados e regressivos são ignorados. Intervalos positivos inferiores a 1 ms só atualizam P, conservando o último histórico de I/D. A ausência da bola libera o estado I/D e produz neutro temporário ou STOP. Se a perda exceder o timeout configurado, a sessão termina.

A compensação mínima opcional se aplica apenas a P puro, gradualmente entre 3 e 8 px e dentro do limite configurado. Assim os modos com integral ou derivada preservam sua saída matemática.

| Parâmetro | Valor inicial e unidade |
|---|---|
| `mode, kp, ki, kd, tf` | PIDF, 0,02 p.p./px, 0, 0, 0,1 s |
| `limit, correction_min` | 2 p.p. por eixo e 0 p.p. |
| `invert` | verdadeiro em X/Y, preservado do perfil anterior |
| `course_min, course_max, pwm_max` | 10%, 90%, 90% por motor, preservados do perfil anterior |
| `matrix` | `[[0,1],[1,0]]`, linhas M1/M3 e colunas X/Y |
| `neutral` | `[40,40]` em porcentagem de curso, ordem M1/M3 |
| `loss_timeout_s` | 0,32 s, configurável entre 0,05 e 0,32 s |
| `loss_behavior` | `neutral_then_stop`; `stop_on_missing` encerra ao primeiro resultado recente sem bola |
| `ack_timeout_s` | 0,15 s; faixa 0,02–0,30 s aplicada ao abrir a porta |

## Cascata dos motores

A posição de curso em graus vale `θ = sinal × (contagem − mínimo) × (360/1280)`; o alvo em graus vale `alvo_pct × curso_graus / 100`. Cada malha usa seu período independente. A posição em 20 ms usa PIDF com Kp em s⁻¹, Ki em s⁻², Kd adimensional, Tf em s e saída em °/s. Suas equações seguem as de cima com `T=0,020`, derivada sobre posição e sem o sinal extra da malha externa. O incremento integral usa `Ki×T` e a recorrência derivativa usa `α=Tf/(Tf+T)` e `β=Kd/(Tf+T)`. Os coeficientes são recalculados após `CFG`. A saturação de velocidade e a faixa de variação possível por aceleração limitam a integração. Atraso grave repõe o histórico derivativo e a base temporal do PI.

O PI de velocidade usa a direção da referência e o erro de magnitude `|v_ref| − direção × v_med`. Kp=0,20 em %PWM/(°/s), Ki=0,30 em %PWM/°. Integra `Ki × Δt × erro` no período nominal de 20 ms; a primeira amostra ou um intervalo acima de duas amostras ignora o incremento. A referência passa por um limite de distância, velocidade, aceleração e PWM; frações do incremento que não alteram esforço útil são bloqueadas. A lógica observa saída PWM quantizada, freio e retenção de reversão. A saída no sentido da referência preserva a política elétrica existente. As janelas de velocidade permanecem 50 ms para controle e 150 ms para a estimativa longa.

`FRICTION` carrega `[partida+, partida−, sustentação+, sustentação−]` por motor. Zero representa medição ausente. Os limiares específicos de atrito permanecem em arquivo separado. A sustentação ausente usa a base de 13% PWM e a compensação de partida medida é introduzida e retirada suavemente em 100 ms. O feedforward cai continuamente perto da referência zero. A assistência integral anterior ainda existe na rotina lenta de calibração.

O modo BALL_ACTIVE entra em freio a uma contagem do alvo e sai acima de duas, sem temporização de 150 ms. O perfil estático conserva a regra anterior. A centralização usa o neutro configurado e o perfil estático. No rastreamento a estimativa de atraso de frenagem da limitação por distância é 50 ms, enquanto o estático mantém 100 ms. A trajetória, o limite de curso, o freio por fins de curso e a reversão de 30 ms continuam atuando. Os tetos em BALL permanecem 20 °/s e 60 °/s².

| Estado L298 | EN | IN positivo | IN negativo |
|---|---:|---:|---:|
| COAST | 0 | 0 | 0 |
| BRAKE | 1 | 1 | 1 |
| DRIVE_FORWARD | PWM a 20 kHz | 1 | 0 |
| DRIVE_REVERSE | PWM a 20 kHz | 0 | 1 |

Os GPIOs permanecem os do mapa em `motor_control.c` e `imu_test.c`. O RP2040 desabilita Enable antes de escrever novas direções. Encoders e segurança permanecem em 200 Hz. O Core 0 executa encoder, controle e IMU. O Core 1 publica USB em 25 Hz normalmente, com pacote de estado e falha prioritário, e recebe comandos a cada iteração nominal de 2 ms. Os mutexes liberam os snapshots antes de formatar ou transmitir. O watchdog só é alimentado com ambos os motores e ambos os encoders atualizados. O build pode habilitar hooks de falha de heap e de stack.
