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
K_i = 0.64119;
p1 = -2.6850 + 9.2515j;
p2 = -1.4655 + 7.3810j;
disp("Planta ideal")
P_ideal = zpk([0 4/Ts -3], [p1 conj(p1) p2 conj(p2)], K_i)

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

%% Matrices del espacio de estados: Convertir el modelo identificado a espacio de estados (A,B,C,D)
sys_ss = ss(sys_tfest);
A = sys_ss.A;
B = sys_ss.B;
C = sys_ss.C;
D = sys_ss.D;

disp('Matriz A:'); disp(A)
disp('Matriz B:'); disp(B)
disp('Matriz C:'); disp(C)
disp('Matriz D:'); disp(D)

%% Realimentacion de Estados

%
Aa = [A zeros(3, 1); -C 0]
Ba = [B; -D]

%Hago acker con los polos complejos conjugados en donde en un lugar donde
%sean estables ya que no se pide requerimientos y agrego un polo en cero
%para tener accion integral
%Tomo polos random CC en el semiplano izquierdo
ceda = 0.9
sigma = 5 
wn = sigma/ceda
wd = sqrt(wn^2 - sigma^2)

%Defino las ganancias
%Ka = acker(Aa, Ba, [-sigma + wd*i, -sigma - wd*i, -1, -10])
Ka = acker(Aa, Ba, [-4, -4, -400, -4])

%Obtengo K y ki
K = Ka(1:3)
kI = Ka(end)
%% Observador 

%Comprobación de observabilidad
n = size(A,1);
if rank(obsv(A,C)) ~= n
    error('Sistema NO observable: rank(obsv(A,C)) = %d (debe ser %d)', rank(obsv(A,C)), n);
end

% 2) Selección automática de polos del observador (si K existe usa polos cerrados)
if exist('K','var')
    % polos del lazo cerrado con realimentación de estados
    pc = eig(A - B*K);
    % elegir observador ~ 5 veces más rápido (factor ajustable)
    factor = 10;
    p_obs = factor * pc;
    % Si hay polos complejos aparecen pares; keep size n
    % En caso de que el escalado produzca polos con parte real positiva, forzamos negativo
    for ii=1:length(p_obs)
        if real(p_obs(ii)) >= 0
            p_obs(ii) = -abs(real(p_obs(ii))) - ii; % desplazar a izquierda si algo raro
        end
    end
else
    % fallback: polos "manuales" rápidos en semiplano izquierdo
    p_obs = [-8 -8 -500];
end

% Asegurar que p_obs tenga longitud n
if length(p_obs) ~= n
    % si hay más/menos, tomar los n valores con mayor parte real negativa
    p_obs = sort(p_obs, 'ComparisonMethod','real');
    p_obs = p_obs(1:n);
end

% 3) Cálculo de la ganancia del observador (L)
Lo = place(A', C', p_obs)';    % Lo es (n x ny) -> aquí ny = 1
disp('Polos elegidos para observador completo:');
disp(p_obs.');
disp('Ganancia observador Lo:');
disp(Lo);

% Ecuación del estimador continuo (uso conceptual):

%dot_xhat = A*x + B*u + Lo*(y - C*x);
