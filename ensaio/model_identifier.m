
%% ========================================================================
% IDENTIFICACAO DE MOTOR DC - RP2040 + FreeRTOS
% Ensaio PRMS bipolar - System Identification Toolbox
%
% Entrada : PWM [%] de -100 a +100
% Saida 1 : Velocidade angular [rad/s]
% Saida 2 : Posicao angular [rad]
%
% ========================================================================

clear;
clc;
close all;

%% ------------------------------------------------------------------------
% 1. Importacao dos dados exportados pela Raspberry Pi Pico
% -------------------------------------------------------------------------

filename = 'C:\Users\herog\vscode\micro\Projeto_Mesa_De_Stewart\ensaio\dados_motor1.txt';

fid = fopen(filename, 'r');

if fid == -1
    error('Nao foi possivel abrir o arquivo.');
end

% Ignora os comentarios e o cabecalho.
% Le apenas linhas com cinco valores numericos.

raw = zeros(2000, 5);
n = 0;

while ~feof(fid)

    line = strtrim(fgetl(fid));

    values = sscanf(line, '%f,%f,%f,%f,%f');

    if numel(values) == 5

        n = n + 1;

        if n > size(raw, 1)
            raw(end + 1000, :) = 0;
        end

        raw(n, :) = values(:)';

    end

end

fclose(fid);

if n < 100
    error('Poucas amostras validas encontradas.');
end

raw = raw(1:n, :);

%% ------------------------------------------------------------------------
% 2. Extracao e conversao das variaveis
% -------------------------------------------------------------------------

%rpm[i] = (theta_encoder[i]-theta[i-1])/(t[i]-t[i-1]) * 60/(2*pi) --------%

t = raw(:, 1);
pwm = raw(:, 2);
theta_encoder = raw(:, 3);
rpm_encoder = raw(:, 4);
counts = raw(:, 5);

% Corrige a convencao de sentido do encoder.
% PWM positivo passa a produzir velocidade positiva.

theta = -theta_encoder;
rpm = -rpm_encoder;

% Converte RPM para rad/s.

omega = rpm * 2*pi/60;

%omega = zeros(size(theta));------
%omega(2:end) = diff(theta) ./ diff(t);------

% Periodo de amostragem medido.

Ts = median(diff(t));
Fs = 1/Ts;

fprintf('\n====================================\n');
fprintf('DADOS DO EXPERIMENTO\n');
fprintf('====================================\n');
fprintf('Amostras       : %d\n', n);
fprintf('Duracao total        : %.3f s\n', t(end)-t(1));
fprintf('Periodo medio entre amostras  : %.6f s\n', mean(diff(t)));
fprintf('Frequencia     : %.2f Hz\n', Fs);
fprintf('PWM minimo     : %.1f %%\n', min(pwm));
fprintf('PWM maximo     : %.1f %%\n', max(pwm));
fprintf('RPM minimo     : %.2f\n', min(rpm));
fprintf('RPM maximo     : %.2f\n', max(rpm));

%% ------------------------------------------------------------------------
% 3. Visualizacao das medicoes experimentais
% -------------------------------------------------------------------------

figure('Name', 'Dados experimentais');

subplot(3,1,1);
stairs(t, pwm, 'LineWidth', 1.2);
ylabel('PWM [%]');
title('Entrada aplicada ao motor');
grid on;

subplot(3,1,2);
plot(t, rpm, 'LineWidth', 1.2);
ylabel('RPM');
title('Velocidade angular');
grid on;

subplot(3,1,3);
plot(t, theta, 'LineWidth', 1.2);
xlabel('Tempo [s]');
ylabel('Posicao [rad]');
title('Posicao angular acumulada');
grid on;

%% ------------------------------------------------------------------------
% 4. Criacao do objeto iddata
% -------------------------------------------------------------------------

% Entrada: duty cycle em percentual.
% Saida: velocidade angular em rad/s.
%
% Os valores sao mantidos sem normalizacao ou detrend.
% Assim, o ganho identificado preserva as unidades fisicas.

data = iddata(omega, pwm, Ts, ...
    'InputName', 'PWM', ...
    'OutputName', 'Velocidade', ...
    'InputUnit', '%', ...
    'OutputUnit', 'rad/s', ...
    'TimeUnit', 's');

%% ------------------------------------------------------------------------
% 5. Separacao dos conjuntos de identificacao e validacao
% -------------------------------------------------------------------------

% Primeiros 70% para estimar os parametros.
% Ultimos 30% para verificar a capacidade de predicao.

N_est = floor(0.70*n);

data_est = data(1:N_est);
data_val = data(N_est+1:end);

%% ------------------------------------------------------------------------
% 6. Identificacao - primeira ordem
% -------------------------------------------------------------------------

% Modelo:
%
%           K
% G(s) = --------
%        tau*s + 1
%
% A funcao tfest estima os parametros a partir dos dados.

G1 = tfest(data_est, 1, 0);

disp(' ');
disp('====================================');
disp('MODELO DE PRIMEIRA ORDEM');
disp('====================================');
G1

%% ------------------------------------------------------------------------
% 7. Identificacao - primeira ordem com um zero
% -------------------------------------------------------------------------

% Modelo:
%
%           b1*s + b0
% G(s) = ---------------
%        tau*s + 1
%
% A funcao tfest estima os parametros a partir dos dados.

G2 = tfest(data_est, 1, 1);

disp(' ');
disp('====================================');
disp('MODELO DE PRIMEIRA ORDEM COM ZERO');
disp('====================================');
G2

%% ------------------------------------------------------------------------
% 8. Identificacao - segunda ordem
% -------------------------------------------------------------------------

% Modelo:
%
%                 b0
% G(s) = ---------------------
%        s^2 + a1*s + a0

G3 = tfest(data_est, 2, 0);

disp(' ');
disp('====================================');
disp('MODELO DE SEGUNDA ORDEM');
disp('====================================');
G3

%% ------------------------------------------------------------------------
% 9. Identificacao - segunda ordem com um zero
% -------------------------------------------------------------------------

% Modelo:
%
%              b1*s + b0
% G(s) = ---------------------
%        s^2 + a1*s + a0

G4 = tfest(data_est, 2, 1);

disp(' ');
disp('====================================');
disp('MODELO DE SEGUNDA ORDEM COM ZERO');
disp('====================================');
G4

%% ------------------------------------------------------------------------
% 10. Identificacao - segunda ordem com dois zeros
% -------------------------------------------------------------------------

% Modelo:
%
%        b2s^2 + b1*s + b0
% G(s) = ---------------------
%        s^2 + a1*s + a0

G5 = tfest(data_est, 2, 2);

disp(' ');
disp('====================================');
disp('MODELO DE SEGUNDA ORDEM COM DOIS ZEROS');
disp('====================================');
G5

%% ------------------------------------------------------------------------
% 11. Comparacao nos dados de identificacao
% -------------------------------------------------------------------------

figure('Name', 'Identificacao');

compare(data_est, G1, G2, G3, G4, G5);
grid on;

%% ------------------------------------------------------------------------
% 12. Validacao em dados nao usados no ajuste
% -------------------------------------------------------------------------

figure('Name', 'Validacao');

compare(data_val, G1, G2, G3, G4, G5);
grid on;

%% ------------------------------------------------------------------------
% 13. Analise das funcoes de transferencia
% -------------------------------------------------------------------------

figure('Name', 'Bode dos modelos');

bode(G1, G2, G3, G4, G5);
grid on;
legend('Primeira ordem', ...
       'Primeira ordem com zero', ...
       'Segunda ordem', ...
       'Segunda ordem com zero', ...
       'Segunda ordem com dois zeros');

%% ------------------------------------------------------------------------
% 14. Resposta ao degrau unitario
% -------------------------------------------------------------------------

figure('Name', 'Resposta ao degrau');

step(G1, G2, G3, G4, G5);
grid on;
legend('Primeira ordem', ...
       'Primeira ordem com zero', ...
       'Segunda ordem', ...
       'Segunda ordem com zero', ...
       'Segunda ordem com dois zeros');

%% ------------------------------------------------------------------------
% 15. Construcao da planta de posicao
% -------------------------------------------------------------------------

% A posicao e a integral da velocidade:
%
% Theta(s) = Omega(s)/s
%
% Portanto:
%
% G_pos(s) = G_vel(s)/s

s = tf('s');

Gpos1 = tf(G1)/s;
Gpos2 = tf(G2)/s;
Gpos3 = tf(G3)/s;
Gpos4 = tf(G4)/s;
Gpos5 = tf(G5)/s;

disp(' ');
disp('====================================');
disp('PLANTAS DE POSICAO');
disp('====================================');

disp('Posicao - primeira ordem integrada:');
Gpos1

disp('Posicao - primeira ordem com zero integrada:');
Gpos2

disp('Posicao - segunda ordem integrada:');
Gpos3

disp('Posicao - segunda ordem com zero integrada:');
Gpos4

disp('Posicao - segunda ordem com dois zeros integrada:');
Gpos5

%% ------------------------------------------------------------------------
% 16. Selecao provisoria para o PID Tuner
% -------------------------------------------------------------------------

% Inicialmente selecionamos a primeira ordem integrada.
% Altere para Gpos2 ou Gpos3 se a validacao justificar.

G_motor_velocidade = tf(G1);

G_motor_posicao = Gpos1;

disp(' ');
disp('PLANTA SELECIONADA PARA CONTROLE:');
G_motor_posicao

%% ------------------------------------------------------------------------
% 18. Abrir as ferramentas do MATLAB
% -------------------------------------------------------------------------

% Interface de identificacao:
% systemIdentification
%
% Interface de sintonia do controlador:
% pidTuner(G_motor_posicao, 'PID')

disp(' ');
disp('Identificacao concluida.');
disp('Para abrir o PID Tuner, execute:');
disp('pidTuner(G_motor_posicao, ''PID'')');
