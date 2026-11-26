# Compilação, testes e bancada

Execute os comandos a partir de `imu_test/`. No Windows o SDK, GCC ARM e Ninja instalados pela extensão Raspberry Pi Pico podem estar em `$env:USERPROFILE/.pico-sdk`. A compilação verificada usou SDK Pico 2.3.1 e GCC ARM 15.2.1.

```powershell
python -m pip install -r requirements.txt
cmake -S . -B build -G Ninja -DPICO_BOARD=pico
cmake --build build
python -B -m unittest discover -s tests -v
python -B tests/run_firmware_tests.py gcc
python -B visualizar_imu.py --mock
```

Com um SDK fora do ambiente, passe `-DPICO_SDK_PATH=<pasta-do-sdk>` ao configurar. Em uma instalação sem `cmake` ou Ninja no PATH, execute o binário da extensão e passe `-DCMAKE_MAKE_PROGRAM=<caminho-do-ninja>` e `-DPICO_TOOLCHAIN_PATH=<pasta-do-gcc-arm>`. Use `-DSTEWART_FREERTOS_DIAGNOSTICS=ON` em um diretório de build separado para habilitar os hooks. O teste C aceita `clang` ou `zig cc` no lugar de `gcc`. O teste Tk é omitido pelo unittest quando o ambiente carece de servidor gráfico.

O build padrão gera `build/imu_test.uf2`. A instalação do UF2 é feita com a Pico em BOOTSEL e a alimentação dos motores desligada. Depois da gravação, execute `python -B visualizar_imu.py` e selecione a porta USB da Pico. Em modo `--mock`, a interface usa telemetria e protocolo simulados.

## Diagnóstico sem acionamento

```powershell
python -B diagnostico.py serial --mock --count 1000 --output serial-sim.csv
python -B diagnostico.py bola --output bola-sintetica.csv
python -B diagnostico.py bola --input observacoes.csv --target-x 320 --target-y 240 --output bola.csv
```

O CSV de entrada da bola usa `capture_s,x_px,y_px`, em tempo monotônico ou relativo e com pixels da imagem processada. Campos de posição vazios representam perda da detecção. O diagnóstico roda a mesma malha PID da GUI. Os arquivos de saída mostram os termos de cada eixo, saturação e referências M1/M3.

Na bancada, trave a plataforma mecanicamente e mantenha a alimentação de potência desligada. Feche a GUI e execute `python -B diagnostico.py serial --port COM4 --count 1000 --output serial-pico.csv` na porta correspondente. BALLTEST2 desabilita os Enables durante o diagnóstico. Registre p95, quantidade de ACK, timeouts e NACK. Busque p95 < 20 ms e nenhum comando válido sem resultado explicado. Repita com `--telemetry-hz 200` se precisar avaliar carga. Envie fragmentos ou linhas inválidas em uma sessão de diagnóstico e confirme a recuperação no comando seguinte.

## Ensaios com o operador na bancada

1. Confirme a sequência EN e IN com osciloscópio, primeiro com a potência isolada. Verifique inicialização, freio, coast, troca de sentido com Disable antes da troca, reversão e reset. Com alimentação e movimento autorizados, monitore corrente da L298 e teste STOP e perda de link dentro da faixa.
2. Observe os encoders ao girar manualmente cada eixo de saída uma volta, com a potência desligada. Compare a diferença absoluta com 1280 contagens e verifique a polaridade dos dois sentidos. Confirme sinais A/B, consistência de `min/max`, porcentagens e resposta ao encoder desconectado. Calibre M1/M3, execute Home e confirme 40%. Rode o nivelamento com IMU e capture os neutros na aba Câmera.
3. Teste degraus de ±1°, pequenos próximos da resolução e maiores dentro do curso. Teste seno, cosseno, rampa e reversões a partir da aba Acionamento. Compare subida, acomodação, sobressinal, erro final, PWM, velocidade e tempo em BRAKE no CSV de motores. Compare séries com o mesmo curso, alimentação, carga e janela de tempo.
4. Na identificação de atrito, teste partida por motor e sentido em posições intermediárias do curso. Para sustentação, marque Sustentação, inicie o movimento e reduza o PWM em passos. Mantenha cada nível por pelo menos 300 ms e aguarde o relatório da última magnitude que sustentou movimento. O ensaio exige pelo menos duas contagens no sentido testado por janela. Anote posição inicial, carga, tensão e direção. Salve os limiares somente após comparar repetições.
5. Reproduza uma série de referências em pares com a CLI de motores. Use alvos dentro do perfil, instantes iniciando em zero e intervalos de 50 a 300 ms. Faça um ensaio simulado antes do ensaio físico. A série termina com STOP. O CSV contém registros de alvo, estado e motor, com seus campos em JSON para preservar os esquemas.
6. Com câmera e placa conectadas, use o perfil PIDF de teste. Compare o sentido X→M3 e Y→M1 com pequenos deslocamentos e observe as curvas de erro e curso. Registre idade do quadro, taxas e termos. Oculte a bola, remova o setpoint, provoque perda de comunicação em posição segura e confirme as transições e a necessidade de rearme. Identifique a matriz de acoplamento apenas após medir dois eixos independentes.

Exemplo de `serie.csv`:

```csv
t_s,m1_pct,m3_pct
0,40,40
0.05,41,39
0.10,39,41
0.15,40,40
```

```powershell
python -B diagnostico.py motores --mock --input serie.csv --output motores-sim.csv
python -B diagnostico.py motores --port COM4 --autorizar-movimento --input serie.csv --output motores-pico.csv
```

Para investigar o atraso com mais resolução, use `--telemetry-hz 200` no ensaio de motores e compare com a execução a 25 Hz. Os gráficos e logs mantêm os dois relógios com suas unidades próprias.

## Resultados obtidos

| Camada | Validação |
|---|---|
| Python | 13 testes passaram, incluindo Tk com Pico simulada, P/PI/PID/PIDF, modos de perda e reset, transformação M1/M3, ACK atrasado/ausente, STOP, reconexão e telemetria antiga/nova. |
| Firmware em host | O teste C executou o parser real, centralização, ACK/NACK, limites, timeout, rearme, captura dinâmica, GPIO lógico, reversão, feedforward assimétrico, estimativas de velocidade, supervisão do watchdog e encoder atrasado. Foi compilado com Zig cc e `-Werror`. |
| RP2040 | Builds padrão e com hooks FreeRTOS compilaram e vincularam. O UF2 padrão tem 221.184 bytes. |
| Simulação USB | 1.000 de 1.000 comandos confirmados pelo mock, com p95 de RTT de 0,13 ms na execução final e 0,23 ms em uma repetição sob carga do processo de validação. Essa medição de software usa transporte em memória, sem latência física de USB CDC. |
| Diagnóstico da bola | Com erro X de +10 px e Y de −4 px, a polaridade original produziu M1=40,08% e M3=39,80%. A reprodução simulada de referências dos motores também terminou com sucesso. |
| Hardware | A enumeração encontrou a Pico na COM4. A placa permaneceu com o firmware que já estava instalado. Sem ensaio físico, ficam pendentes RTT USB, tempos do RTOS sob carga, corrente, polaridade e resolução medidas, qualidade do YOLO com a câmera, aderência ao alvo e estabilidade com bola. |
