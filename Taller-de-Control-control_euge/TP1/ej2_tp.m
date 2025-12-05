clear all; close all;

% Defino el sistema
A = [0 1 0 0; 0 -0.1818 2.6724 0; 0 0 0 1; 0 -0.4545 31.1818 0];
B = [0; 1.8182; 0; 4.5455];
C = [1 0 0 0; 0 0 1 0];
D = 0;

% Amplio el sistema para realizar acción integral 
A_monio = [A, zeros(4, 1); -C(1, :), zeros(1,1)];
B_monio = [B; zeros(1, 1)];
C_monio = [C, zeros(2 ,1)];

% Defino polos deseados
plc = [-5.6+2i -5.6-2i -9 -9.00001 -10];

% Obtengo ganancia para realimentacion de estados
K = -place(A_monio, B_monio, plc);
Kr = K(:,1:4);
Ka = K(:,5); 
eig(A_monio+B_monio*K)

yref = 0.2;


%% inciso b

% Defino matriz de transformación
T = [1 0 0 0; 0 0 1 0; 0 1 0 0; 0 0 0 1];
At = T*A*inv(T);
Bt = T*B;
Ct = C*inv(T);

% Defino submatrices
A11 = At(1:2, 1:2);
A12 = At(1:2, 3:4);
A21 = At(3:4, 1:2);
A22 = At(3:4, 3:4);

B1 = Bt(1:2,:);
B2 = Bt(3:4,:);

% Defino polos del observador
plo = [-15 -15.0001];

% Obtengo matriz de ganancia para el observador
Lred = place(A22', A12', plo)'; 

% Obtengo matrices para el observador reducido
F = A22 - Lred*A12;
G = F*Lred - Lred*A11 + A21;
H = B2 - Lred*B1;

% Matriz auxiliar para medir todas las salidas
Caux = eye(4,4);

%% inciso c
% Matriz para medir una unica salida
C_unica = [1 0 0 0];

% Defino submatrices
A11_2 = At(1:1, 1:1);
A12_2 = At(1:1, 2:4);
A21_2 = At(2:4, 1:1);
A22_2 = At(2:4, 2:4);

B1_2 = Bt(1:1,:);
B2_2 = Bt(2:4,:);

% Defino polos para el observador
plo = [-12 -12.0001, -13];
Lred_2 = place(A22_2', A12_2', plo)'; 

% Defino matrices para el observador reducido
F_2 = A22_2 - Lred_2*A12_2;
G_2 = F_2*Lred_2 - Lred_2*A11_2 + A21_2;
H_2 = B2_2 - Lred_2*B1_2;
