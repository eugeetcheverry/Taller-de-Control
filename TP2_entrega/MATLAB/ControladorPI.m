clc; clear all; close all;
s = tf('s');
opt = bodeoptions;
opt.PhaseWrapping = 'on';        % Para mostrar fase envuelta (útil si hay ±180°)
opt.FreqUnits = 'Hz';            % Opcional: puedes usar 'rad/s' o 'Hz'
opt.Grid = 'on';                 % Agrega grilla
opt.MagUnits = 'dB';             % Magnitud en decibeles
opt.Title.String = 'Diagrama de Bode con fase visible';
w = logspace(-1, 6, 1000);  % De 0.1 a 100 rad/s, 1000 puntos

Ts = 0.02;
T0 = 0.32;

% ------------------- Planta ----------------------------------------------
Planta = -0.1014 * (s * (s - 1.5391)) / (4.715e-06 * s^4 + 0.0008047 * s^3 + 0.04633 * s^2 + 0.2726 * s + 3.259);

% ------------------- Controlador -----------------------------------------
k0 = 0.8;
kp = 0.45*k0
ki = 0.54*k0 / T0
C = kp + ki / s

figure();
bode(C*Planta, w, opt)
title('Bode de la planta con controlador PI');


