clc; clear all; close all;

% Cargar datos
data = readmatrix("C:\Users\camid\OneDrive\Documentos\Control1C2025\TP2_control\TP2\MATLAB\DATOS_2605_cambios2.csv");

Ts = 0.02;
 
% Extraer señales y pasar a fila
theta0 = data(:,2)';
phi0 = data(:,3)';

% Calcular derivadas (orden 1 a 4 para theta, 1 y 2 para phi)
theta1 = diff(theta0)/Ts;
theta2 = diff(theta1)/Ts;
theta3 = diff(theta2)/Ts;
theta4 = diff(theta3)/Ts;
theta5 = diff(theta4)/Ts;

phi1 = diff(phi0)/Ts;
phi2 = diff(phi1)/Ts;
phi3 = diff(phi2)/Ts;

% Cortar todos los vectores al tamaño mínimo compatible
N = length(theta5);

theta0 = theta0(1:N);
theta1 = theta1(1:N);
theta2 = theta2(1:N);
theta3 = theta3(1:N);
theta4 = theta4(1:N);
theta5 = theta5(1:N);

phi1 = phi1(1:N);
phi2 = phi2(1:N);
phi3 = phi3(1:N);

phi0 = phi0(1:N);

% Matriz de regresores: [4 polos, 2 ceros]
% orden: [theta1 theta2 theta3 theta4 phi1 phi2]
A = [theta0', theta1', theta2', theta3', theta4', phi1', phi2', phi3'];

b = phi0';

% Mínimos cuadrados directo sin normalizar
X = (A'*A) \ (A'*b);

% Construcción de la función de transferencia
% Denominador (polos): 1 + a1 s + a2 s^2 + a3 s^3 + a4 s^4
% NOTA: orden invertido para tf en MATLAB es [1 a4 a3 a2 a1], por eso se acomoda así
den = [X(5), X(4), X(3), X(2), X(1)];

% Numerador (ceros): 1 + b1 s + b2 s^2
num = [-X(8), -X(7), -X(6), 1];

% Función de transferencia discreta
P_est_d = tf(num, den, Ts)

% Mostrar polos y ceros discretos
ceros = zero(P_est_d);
polos = pole(P_est_d);

disp('Polos discretos:');
disp(polos);

disp('Ceros discretos:');
disp(ceros);
%%
clc; clear all; close all;
s = tf('s');
opt = bodeoptions;
opt.PhaseWrapping = 'on';        % Para mostrar fase envuelta (útil si hay ±180°)
opt.FreqUnits = 'Hz';            % Opcional: puedes usar 'rad/s' o 'Hz'
opt.Grid = 'on';                 % Agrega grilla
opt.MagUnits = 'dB';             % Magnitud en decibeles
opt.Title.String = 'Diagrama de Bode con fase visible';
w = logspace(-1, 6, 1000);  % De 0.1 a 100 rad/s, 1000 puntos

Planta = -0.1014 * (s * (s - 1.5391)) / (4.715e-06 * s^4 + 0.0008047 * s^3 + 0.04633 * s^2 + 0.2726 * s + 3.259);

figure();
rlocus(Planta);
title('Root locus de la planta');

figure();
bode(Planta, w, opt);
title('Bode de la panta');

figure();
step(Planta);
title('Respuesta al escalón de la planta')
