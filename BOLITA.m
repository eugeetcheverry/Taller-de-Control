clc;
clear all;
close all;

syms x1 x2 x3 u y;

g=10;
m=0.1;
L=1e-2;
R=4;

f1=x2;
f2=g-x3/(m*x1);
f3=u/L-x3*R/L;
y=x1;

A=jacobian([f1;f2;f3],[x1,x2,x3]);
B=jacobian([f1;f2;f3],u);
C=jacobian(y,[x1,x2,x3]);
D=jacobian(y,u);

x1=1;
x2=0;
x3=m*g*x1;
u= m*g*R*x1;


A = eval(A)
B = eval(B)
C = eval(C)
D = eval(D)

%planta del sist
P= zpk(ss(A,B,C,D))


%controlDOR
k= db2mag(76.3);
CI=zpk([-3.16,-3.16],[0, -1000], -k)


%lazo abierto
L = minreal(P*CI)


% transferencias
T = minreal (L/(1+L))
S = minreal(1/(1 + L))

%%
s=tf('s');

fase = deg2rad(25);
m=1;
p1=3.16;
wgc=p1/tan(fase/(m*2));

fase_dig=deg2rad(5);

Ts =4*tan(fase_dig/2)/wgc; 

H = (1 -s*(Ts/4))/(1+s*(Ts/4));

L=minreal(CI*P*H)








