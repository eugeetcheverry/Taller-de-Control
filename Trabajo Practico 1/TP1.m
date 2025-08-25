clc;
clear all;
close all;

syms x1 x2 x3 u y;

%Constantes
g = 9.8;
m = 0.1;
L = 1e-2;
R = 4;

%Espacio de estados
f1 = x2;
f2 = g - x3/(m*x1);
f3 = u/L - x3*R/L;
y = x1;

%Linealizo
A = jacobian([f1; f2; f3], [x1, x2, x3]);
B = jacobian([f1; f2; f3], u);
C = jacobian(y, [x1, x2, x3]);
D = jacobian(y, u);

%Valores equilirio
x1 = 1;
x2 = 0;
x3 = m*g*x1;
u = m*g*R*x1;

%Evaluo en los puntos
A = eval(A)
B = eval(B)
C = eval(C)
D = eval(D)

%Planta del sistema 
P = zpk(ss(A, B, C, D))

%Defino controlador 
K = db2mag(31);
C = zpk([-3.13, -400], [0, 0, -10], -K)

%Lazo abierto
L = minreal(P*C)

%Transferencias
T = minreal(L/(1 + L))
S = minreal(1/(1 + L))



    
