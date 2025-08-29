clc; clear; close all;

%% 1. Planta original
s = tf('s');
G = ss(zpk((1-s)*(2-s)/((s-5)*(s-4))));  % Planta de orden 2
A = G.A;
B = G.B;
C = G.C;
D = G.D;

n = size(A,1);  % orden del sistema

%% 1b. Test de controlabilidad y observabilidad

Co = ctrb(A, B);
Ob = obsv(A, C);

fprintf('Rango de la matriz de controlabilidad: %d\n', rank(Co));
fprintf('Rango de la matriz de observabilidad:  %d\n', rank(Ob));

if rank(Co) == size(A,1)
    disp('✅ La planta es completamente controlable.');
else
    disp('❌ La planta NO es completamente controlable.');
end

if rank(Ob) == size(A,1)
    disp('✅ La planta es completamente observable.');
else
    disp('❌ La planta NO es completamente observable.');
end


%% 2. Sistema aumentado (para acción integral)
% z_dot = r - y = r - Cx
Aa = [A, zeros(n,1);
     -C, 0];
Ba = [B;
       0];  % El integrador ya no recibe -D

%% 3. Controlador con acción integral
poles_aug = [-2, -2, -2];  % Polos deseados para sistema aumentado
Ka = acker(Aa, Ba, poles_aug);
K  = Ka(1:n);     % Ganancia de estados
kI = Ka(end);     % Ganancia del integrador (positivo)

%% 4. Ganancia de referencia (para error en régimen = 0)
kr = 1 / ( C * inv(A - B*K) * B );

%% 5. Observador de estados
L = acker(A', C', [-4, -4])';  % Polos del observador

%% 6. Ensamble del sistema total (estados: x, z, x_hat)
% Estados: x (2), z (1), x_hat (2) → total 5

% Dinámica:
% x_dot     = A*x - B*K*x_hat - B*kI*z + B*kr*r
% z_dot     = r - C*x
% xhat_dot  = A*xhat - B*K*xhat - B*kI*z + B*kr*r + L*C*(x - xhat)

A_total = [ A,       -B*kI,         -B*K;
           -C,         0,      zeros(1,n);
           L*C,     -B*kI,   A - B*K - L*C ];

B_total = [ B*kr;
            1;
            B*kr ];

C_total = [ C, 0, zeros(1,n) ];
D_total = 0;

%% 7. Sistema en espacio de estados cerrado
sys_cl = ss(A_total, B_total, C_total, D_total);

%% 8. Simulación
t = 0:0.01:10;
r = ones(size(t));  % entrada escalón

[y, t_out] = lsim(sys_cl, r, t);

%% 9. Gráfica
figure;
plot(t_out, y, 'LineWidth', 2);
grid on;
xlabel('Tiempo (s)');
ylabel('Salida y(t)');
title('Respuesta con control integral, observador y seguimiento');
