clc;
clear all;
close all;

%% === CARGA DE DATOS ===
tmp = load('data.mat');
fn = fieldnames(tmp);
ts = tmp.(fn{1});

if ~isa(ts, 'timeseries')
    error('El archivo no contiene un objeto timeseries válido.');
end

t = ts.Time;
dat = squeeze(ts.Data)';
[n, m] = size(dat);
fprintf('Muestras: %d, Canales: %d\n', n, m);

%% === SELECCIÓN DE SEÑALES ===
% Asumimos que el canal 1 es salida y el 3 es entrada según tus resultados previos
theta = double(dat(:,1));
phi = double(dat(:,3));
Ts = mean(diff(t));

% Crear objeto iddata
data = iddata(theta, phi, Ts);

%% === IDENTIFICACIÓN CON TFEST ===
nb = 4;      % orden del numerador
nf = 3;      % orden del denominador
%ioDelay = 1; % retardo posible

sys_tfest = tfest(data, nb, nf);

disp('========================================');
disp('Modelo identificado con tfest:');
sys_tfest

%% === COMPARACIÓN CON DATOS ===
figure;
compare(data, sys_tfest);
title('Comparación entre datos medidos y modelo tfest');
grid on;

%% === POLINOMIOS Y ESTABILIDAD ===
[num, den] = tfdata(sys_tfest, 'v');
poles = roots(den);
disp('Polos del sistema:');
disp(poles);
if all(real(poles) < 0)
    disp('✅ Sistema estable (polos en semiplano izquierdo)');
else
    disp('⚠️ Sistema inestable o con polos en el eje imaginario');
end

%% Recorto datos
t_start = 12.2408;   % tiempo inicial
t_end   = 18.9228;   % tiempo final

% Crear máscara lógica para los tiempos dentro del rango
mask = (t >= t_start) & (t <= t_end);

% Aplicar máscara
t_rec = t(mask);
theta_rec = theta(mask);
phi_rec = phi(mask);

% Desplazar para que empiece desde cero
t_rec = t_rec - t_rec(1);

%% Creo de nuevo el set idata

% Crear objeto iddata
data_rec = iddata(theta_rec, phi_rec , Ts);
sys_tfest = tfest(data_rec, nb, nf);

disp('========================================');
disp('Modelo identificado con tfest y datos recortados:');
sys_tfest

%% === COMPARACIÓN CON DATOS ===
figure;
compare(data, sys_tfest);
title('Comparación entre datos medidos y modelo tfest');
grid on;
%% === POLINOMIOS Y ESTABILIDAD ===

[num, den] = tfdata(sys_tfest, 'v');
poles = roots(den);
disp('Polos del sistema:');
disp(poles);
if all(real(poles) < 0)
    disp('✅ Sistema estable (polos en semiplano izquierdo)');
else
    disp('⚠️ Sistema inestable o con polos en el eje imaginario');
end

step(sys_tfest)

%% Planta ideal
K = 0.64119;
p1 = -2.6850 + 9.2515j;
p2 = -1.4655 + 7.3810j;
disp("Planta ideal")
P_ideal = zpk([0 4/Ts -3], [p1 conj(p1) p2 conj(p2)], K)

%% Controlador PD
K_c = 1;
K_p = 0.3;
K_d = 0.5;
s = tf("s");
disp("Controlador propuesto")
C = K_p + K_d*s
%% Analisis Lazo abierto PD
disp("Transferencia lazo abierto")
L = minreal(P_ideal*C)

