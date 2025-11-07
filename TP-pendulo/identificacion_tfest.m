clc;
clear all;
close all;

%% === CARGA DE DATOS ===
tmp = load('midata.mat');
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
y = double(dat(:,1));
u = double(dat(:,3));
Ts = mean(diff(t));

% Crear objeto iddata
data = iddata(y, u, Ts);

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
