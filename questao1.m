clear all; close all;  clc; 

s = sym('s');
gs = (s^2+s+1)/(s^4+s^3+2*s^2-2);

% extraido de gs
a = [1 1 2 -2];
b = [1 1 1];

% converte para representacao state-space
[A,B,C,D] = tf2ss(b,a);

% polinomio caracteristico = det(si - A)
lambda = sym('lambda');
si = [lambda 0 0;0 lambda 0;0 0 lambda];
pc = det(si - A);

% matrix de controlabilidade
Co = ctrb(A,B);

% num estados nao controlaveis
nc = length(A) - rank(Co); % controlavel

[Abar,Bbar,Cbar,P,k] = ctrbf(A,B,C);
pole = eig(A);

% polos desejados
% separando polos pois usando 5 polos precisaria ter 5 linhas em A
newpole1 = [-2+2*i -2-2*i -8];
k1 = place(A,B,newpole1); % atribuicao de polo malha fechada com realim estado
Acl1 = A - B*k1;
Ecl1 = eig(Acl1);

newpole2 = [-4+4*i -4-4*i -8];
k2 = place(A,B,newpole2); % atribuicao de polo malha fechada com realim estado
Acl2 = A - B*k2;
Ecl2 = eig(Acl2);

% polinomio caracteristico desejado
%pck = (s+8)*(s+2+2*j)*(s+2-2*j)*(s+4+4*j)*(s+4-4*j);

% sistema fechado com polos 1 e 2
syscl1 = ss(Acl1,B,C,D);
syscl2 = ss(Acl2,B,C,D);
