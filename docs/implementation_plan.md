# Plano de implementação

1. P0. Registrar o baseline Git, mapear o software autoral, localizar o toolchain e executar a descoberta de testes.
2. P1. Corrigir escala e estados do PID. Conectar a matriz e os neutros ao controle e separar a aquisição, a inferência, o consumo de observações e o Tk.
3. P2. Versionar BALL. Correlacionar sessão, estados e ACK. Implementar janela unitária, invalidação por STOP e modo sem energia no Enable.
4. P3. Limitar telemetria, publicar falhas e estados com prioridade e registrar os instantes de transmissão, captura e recepção.
5. P4. Separar rastreamento e captura estática com histerese em contagens.
6. P5. Revisar discretização, períodos e saturação. Preservar os ganhos, estimadores e frequências da cascata até medir o sistema. Medir atrito por motor e sentido.
7. P6. Explicitar os comandos da ponte H e verificar as proteções com GPIO e RTOS simulados.
8. P7. Executar a suíte Python e C, compilar as variantes RP2040, medir o loop simulado e publicar o roteiro de bancada.

A ordem foi aplicada em mudanças incrementais no diretório de trabalho. A auditoria mantém as referências ao código anterior. A sequência de validação reproduzível está no [roteiro de bancada](control_system_validation.md).
