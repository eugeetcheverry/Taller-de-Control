clc;
clear all;
close all;
%% Cargar datos
%Cargar mediciones
tmp = load('data.mat');  

%Guardar el objeto timeseries en una variable propia
fn = fieldnames(tmp);%Obtengo el nombre real del objeto
ts_obj = tmp.(fn{1});%Accedo al timeseries

%Extraer tiempos y canales
t_all  = ts_obj.Time;
theta  = squeeze(ts_obj.Data(1,:,:));
%phi    = squeeze(ts_obj.Data(2,:,:));
phi    = squeeze(ts_obj.Data(3,:,:)); 
%% Grafico la respuesta de theta
figure;                 % crea una nueva figura
plot(t_all, theta, 'b') % 'b' es para línea azul
grid on                 % agrega cuadrícula
xlabel('Tiempo [s]')    % etiqueta del eje x
ylabel('\theta(t) [rad]') % etiqueta del eje y (usá rad si es ángulo)
title('Evolución de \theta en función del tiempo')
%% Recorto los datos
t_start = 45.70;   % tiempo inicial
t_end   = 53.6;   % tiempo final

% Crear máscara lógica para los tiempos dentro del rango
mask = (t_all >= t_start) & (t_all <= t_end);

% Aplicar máscara
t_rec = t_all(mask);
theta_rec = theta(mask);
phi_rec = phi(mask);

% Desplazar para que empiece desde cero
t_rec = t_rec - t_rec(1);
%% Grafico nuevo set
figure;                 % crea una nueva figura
plot(t_rec, theta_rec, 'b') % 'b' es para línea azul
grid on                 % agrega cuadrícula
xlabel('Tiempo [s]')    % etiqueta del eje x
ylabel('\theta(t) [rad]') % etiqueta del eje y (usá rad si es ángulo)
title('Evolución de \theta en función del tiempo')

%% Creo el CSV
%matriz = table(theta_rec, phi_rec, t_rec);
%writetable(matriz, 'datos.csv'); 
%% Euler
%Remuestreo uniforme en Ts
%T = median(diff(t_rec));

%theta1 = diff(theta_rec)/T;
%theta2 = diff(theta1)/T;
%theta3 = diff(theta2)/T;
%theta4 = diff(theta3)/T;

%phi1 = diff(phi_rec)/T;
%phi2 = diff(phi1)/T;
%phi3 = diff(phi2)/T;

%N = length(theta4);
% Truncar todos los vectores a N muestras
%theta   = theta_rec(1:N);
%theta1  = theta1(1:N);
%theta2  = theta2(1:N);
%theta3  = theta3(1:N);
%theta4  = theta4(1:N);

%phi = phi_rec(1:N);
%phi1 = phi1(1:N);
%phi2 = phi2(1:N);
%phi3 = phi3(1:N);

%A = [theta, theta1, theta2, theta3, theta4, phi1, phi2, phi3];
%b = phi;
% Mínimos cuadrados directo sin normalizar
%X = (A'*A) \ (A'*b);
%% Obtener transferencias

% Función de transferencia discreta
%disp('Función de transferencia discreta')
%P_dis = tf([-X(8) -X(7) -X(6) 1], [X(5) X(4) X(3) X(2) X(1)], 0.02)

% Funcion de transferencia continua
%disp('Función de transferencia continua')
%P_con = d2c(P_dis, 'zoh') %Evitar simplificacion

%% Cargo los datos ajustados
new_data = readtable('datos_modificado_2p68.csv');
theta_new = new_data.theta_rec;
phi_new = new_data.phi_rec;
t_new = new_data.t_rec;

%% Grafico nuevo set
figure;                 % crea una nueva figura
plot(t_new, theta_new, 'b') % 'b' es para línea azul
grid on                 % agrega cuadrícula
xlabel('Tiempo [s]')    % etiqueta del eje x
ylabel('\theta(t) [rad]') % etiqueta del eje y (usá rad si es ángulo)
title('Evolución de \theta en función del tiempo')

%% Euler
%Remuestreo uniforme en Ts
T = median(diff(t_rec));

theta1 = diff(theta_rec)/T;
theta2 = diff(theta1)/T;
theta3 = diff(theta2)/T;
theta4 = diff(theta3)/T;

phi1 = diff(phi_rec)/T;
phi2 = diff(phi1)/T;
phi3 = diff(phi2)/T;

N = length(theta4);
% Truncar todos los vectores a N muestras
theta   = theta_rec(1:N);
theta1  = theta1(1:N);
theta2  = theta2(1:N);
theta3  = theta3(1:N);
theta4  = theta4(1:N);

phi = phi_rec(1:N);
phi1 = phi1(1:N);
phi2 = phi2(1:N);
phi3 = phi3(1:N);

A = [theta, theta1, theta2, theta3, theta4, phi1, phi2, phi3];
b = phi;
% Mínimos cuadrados directo sin normalizar
X = (A'*A) \ (A'*b);

%% Obtener transferencias

% Función de transferencia discreta
disp('Función de transferencia discreta')
P_d = tf([-X(8) -X(7) -X(6) 1], [X(5) X(4) X(3) X(2) X(1)], 0.02)

% Funcion de transferencia continua
disp('Función de transferencia continua')
P_c = d2c(P_d, 'zoh') %Evitar simplificacion

%% 






