clear all; close all;  clc; 

s = sym('s');
gs = (s^2+s+1)/(s^4+s^3+2*s^2-2);

% extraido de gs
a = [1 1 2 -2];
b = [1 1 1];

% atribuindo um valor para r
r=1;

% converte para representacao state-space
[A,B,C,D] = tf2ss(b,a);

% polinomio caracteristico: det(si - A)
lambda = sym('lambda');
si = [lambda 0 0;0 lambda 0;0 0 lambda];
pol = det(si - A);

% matrix de controlabilidade
Co = ctrb(A,B);
% num estados nao controlaveis
nc = length(A) - rank(Co); % 0 -> controlavel

% matriz de observabilidade
Obs = obsv(A,C);
no = length(A) - rank(Obs); % 0 -> observável

% [Abar,Bbar,Cbar,P,k] = ctrbf(A,B,C);

% polos desejados - sistema aumentado

Al = [A zeros(length(A),1);-C 0]; % horzcat([A;C;zeros(1,3)],zeros(5,2)); % [A zeros(length(A),2);-C 0 0;0 0 0 0 0]
Bl = [B;0];
Cl = [C 0];
El = [zeros(length(A),1);1];
newpoles = [-2+2*i -2-2*i -4+4*i -4-4*i];
k = place(Al,Bl,newpoles);
% malha fechada com realim estado
Acl = Al - Bl*k;
Ecl = eig(Acl);

%N = inv(Cl*inv(-Acl+Bl*k)*Bl);

% polinomio caracteristico desejado
%pck = (s+8)*(s+2+2*j)*(s+2-2*j)*(s+4+4*j)*(s+4-4*j);

% sistema aberto
sysop = ss(Al,Bl,Cl,D);

% sistema fechado com realimentacao
syscl = ss(Acl,Bl,Cl,D);

figure(1)
step(tf(sysop))
figure(2)
step(tf(syscl))

figure(3)
hold on
axis
plot(eig(A),'o','color','b')
plot(eig(Acl),'*','color','r')


