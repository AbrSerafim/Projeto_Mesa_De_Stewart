# Protocolo USB CDC v13

O firmware mantém o handshake legado `HELLO`, que coloca os motores em freio, habilita as proteções e responde `!HELLO,13`. O cliente configura um identificador de vínculo com `BALLLINK2 <link>` e aguarda `!BALLLINK2,<link>`. Um novo HELLO invalida o vínculo e encerra a sessão anterior. O contador de sessão cresce dentro do vínculo e o de referência cresce dentro da sessão. Todos os identificadores são inteiros decimais positivos de 32 bits. Valores de curso seguem ordem **M1, M3**.

| PC → Pico | Resposta e efeito |
|---|---|
| `BALLSTART2 <sessão> <link> <min1> <max1> <pwm1> <neutro1> <min3> <max3> <pwm3> <neutro3>` | `!BALLSTATE2,<sessão>,4` quando começa a centralização, ou `!BALLSTARTERR,<sessão>,<motivo>`. Exige curso calibrado, encoders recentes e ausência de falha. |
| `BALLTEST2 <sessão> <link>` | `!BALLSTATE2,<sessão>,5` com Enable desabilitado e modos de movimento bloqueados. |
| `BALL2 <sessão> <seq> <m1_pct> <m3_pct>` | `!BALLACK,<sessão>,<seq>,<estado>,<m1_pct>,<m3_pct>,<mutex_us>` ou `!BALLNACK,<sessão>,<seq>,<motivo>`. Estado 1 confirma escrita atômica dos dois alvos. Estado 4 confirma apenas recepção durante centralização. Estado 5 confirma o diagnóstico desenergizado. |
| `BALLSTATUS` | `!BALLSTATE2,<sessão>,<estado>`. O firmware também envia a transição e repete o estado a cada 250 ms. |
| `STOP` | `!OK,STOP`, invalidação da janela do PC e freio na Pico. O cliente antepõe o byte CAN (0x18) para cancelar uma linha incompleta. |
| `REARM` | `!OK,REARM` somente em freio, com encoders recentes e sem fins conflitantes. |
| `PING` | Mantém o vínculo. `!ERR,PING,sem_sessao` dispara novo HELLO. PC envia a cada 150 ms. |
| `TELEM 25` ou `TELEM 200` | Seleciona taxa de publicação dos motores. Degrau publica a até 200 Hz automaticamente. Encoder e controle executam em períodos independentes. |

Estados de BALL 0, 1, 2, 3, 4 e 5 correspondem a parado, ativo, timeout, falha, centralização e diagnóstico. `!BALLF2,<sessão>,<motor>,<falha>,<posição_pct>,<alvo_pct>,<pwm_pct>,<velocidade>,<limiar>` fornece contexto da falha. NACK inclui motivo e os identificadores recuperáveis. A Pico rejeita argumento a mais, `NaN`, infinito, sequência antiga, sessão antiga e curso fora de faixa. O firmware verifica ambas as referências antes da atualização. A mesma sequência jamais atualiza alvos duas vezes. Identificadores ou referências antigos não prorrogam o watchdog.

Framing ASCII por LF, com CR opcional antes de LF. O RX da Pico aceita até 191 bytes por linha e fragmentos com intervalo de até 100 ms. Linha longa ou byte inválido é descartado até LF e gera `!RXERR,frame`. CAN também recupera o receptor imediatamente. O receptor do PC aceita linhas de até 512 bytes e reagrupa fragmentos.

O PC guarda um comando em voo e a referência mais recente aguardando envio. Uma referência local com idade superior a 120 ms expira. Cada tentativa recebe um resultado, incluindo timeout, cancelamento, perda de conexão, substituição, rejeição ou confirmação. ACK tardio é contabilizado e descartado. O PC usa timeout de ACK inicial de 150 ms, timeout de início de 400 ms e watchdog de referência da Pico de 450 ms. O vínculo tem watchdog de 1 s. Esses valores preservam as janelas de partida e segurança do baseline e precisam ser comparados com dados físicos, sem extrapolar a medição em memória.

`!COMMDIAG` informa contadores RX inválidos e de comandos, tempo do snapshot IMU em µs, formatação em µs, chamada TX em µs, máximo de publicação de motores em µs no último intervalo e stack mínimo livre em words da tarefa de comunicação. `!MOTORDIAG` informa deadlines perdidos, jitter máximo em µs e stack mínimo em words, primeiro M1 depois M3. O tempo de espera por mutex em ACK é uma medição local da Pico. A curva de encoder e referência usa o relógio de recepção no PC e o CSV inclui ainda o timestamp de amostragem em microssegundos na Pico. RTT inclui escrita, transporte, firmware e leitura. Comparações de relógios precisam de sincronização adicional.

Formatos de `CFG`, PING e telemetria principal são preservados para o firmware v13. `@` inclui duas colunas novas após a referência de velocidade, a origem do degrau em graus e o estado da ponte 0/1/2/3 para COAST/BRAKE/DRIVE_FORWARD/DRIVE_REVERSE. `FRICTION` aceita o formato anterior de dois limiares e o formato de quatro limiares `m partida+ partida− sustentação+ sustentação−`. O modo de teste de sustentação usa `PWMSTART m sentido 1` e retorna estado 4 em `!PWMTEST`. `STEP m delta_graus` permite degrau assinado e `STEP m` conserva +1°.
