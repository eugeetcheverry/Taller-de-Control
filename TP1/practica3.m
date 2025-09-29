clc; clear; close all;

% Parámetros verdaderos de la planta continua
K = 2;   
p = 5;   
Ts = 0.1;   % tiempo de muestreo

% Planta continua
s = tf('s');
P = -K/(s+p);

% Discretización con retención de orden cero (ZOH)
Pd = c2d(P, Ts, 'zoh');

disp('Planta discreta real:')
Pd

% Simulación de la respuesta a un escalón
N = 100;                   % muestras
u = 0.1*ones(N,1);         % entrada escalón
t = (0:N-1)'*Ts;           % tiempo
y = lsim(Pd,u,t);          % salida real

% Construcción de matrices para mínimos cuadrados
Phi = [y(1:end-1) u(1:end-1)];
Y = y(2:end);

% Estimación de parámetros (theta1 = a, theta2 = b)
theta = (Phi'*Phi)\(Phi'*Y);

a_est = theta(1);
b_est = theta(2);

fprintf('Parámetros estimados:\n');
fprintf('a = %.4f\n', a_est);
fprintf('b = %.4f\n', b_est);

% Comparación de la salida reconstruida con los parámetros estimados
y_hat = zeros(N,1);
for k = 1:N-1
    y_hat(k+1) = a_est*y_hat(k) + b_est*u(k);
end

figure;
plot(t,y,'b','LineWidth',2); hold on;
plot(t,y_hat,'r--','LineWidth',2);
grid on
xlabel('Tiempo [s]')
ylabel('Salida')
legend('Salida real','Modelo estimado')
title('Identificación por mínimos cuadrados')

%% --- Ajuste usando fit (modelo exponencial discreto) ---
% Queremos ajustar la salida y(t) con un modelo del tipo:
% y(t) = A*(1 - exp(-alpha*t))

ftype = fittype('A*(1 - exp(-alpha*x))', 'independent','x','coefficients',{'A','alpha'});
fit_result = fit(t, y, ftype);

A_fit = fit_result.A;
alpha_fit = fit_result.alpha;

fprintf('\nParámetros ajustados con fit:\n');
fprintf('A = %.4f\n', A_fit);
fprintf('alpha = %.4f\n', alpha_fit);

% La planta continua equivalente sería del tipo: P(s) = -K/(s+p)
% Donde: K ≈ A_fit * alpha_fit, p ≈ alpha_fit
K_fit = A_fit * alpha_fit;
p_fit = alpha_fit;

P_fit = -K_fit/(s+p_fit);

disp('Planta continua aproximada obtenida con fit:')
P_fit

% Graficar comparación del ajuste
y_fit = A_fit*(1 - exp(-alpha_fit*t));

figure;
plot(t,y,'b','LineWidth',2); hold on;
plot(t,y_fit,'m--','LineWidth',2);
grid on
xlabel('Tiempo [s]')
ylabel('Salida')
legend('Salida real','Ajuste con fit')
title('Ajuste de la respuesta temporal con fit')
