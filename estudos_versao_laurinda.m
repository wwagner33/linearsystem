clear all; close all; clc;


% extraido de gs = n/d
d = [1 1 2 -2]; %denominador - polinomio caracteristico
n = [1 1 1]; %numerador - polinomio minimo (?)

% converte para representacao state-space
[A, B, C, D] = tf2ss(n, d);
sys = tf(n,d);

cp = charpoly(A);%polinomio caracteristico do sistema

CM = ctrb(A,B); %calcula Matriz de Controlabilidade

% if (eq(rank(CM),length(A)) == 1) %testa controlabilidade
%     "Sistema é Controlável"
% else
%     "Sistema não é controlável"
% end

%calcula K

[Abar,Bbar,Cbar,P] = ctrbf(A,B,C); %gera as matrizes barradas e o P

CMbar = ctrb(Abar,Bbar); %gera matriz de controlabilidade barrada

%calcula polinomio caracteristico de malha fechada para os polos -2+2j,-2-2j,-4+4j,-4-4j
syms s;

deltaK = (s-2+2j)*(s-2-2j)*(s-4+4j)*(s-4-4j);%polinomio caracteristico desejado MF

cpk = sym2poly(deltaK); %extrais os coeficientes do polinomio caracteristico e coloca num vetor

alfa_sys = [d(2:3) 0 d(4)]; %alfas do provenientes do denominador de G(s)

alfa_k = cpk(2:5); %alfas do polinomio caracteristico desejado

Kbar = [(alfa_k(1) - alfa_sys(1)) (alfa_k(2) - alfa_sys(2)) (alfa_k(2) - alfa_sys(2)) (alfa_k(3) - alfa_sys(4))];

K = Kbar*CMbar*inv(CM);



% sum(k)
%[sysr,u] = minreal(sys,tol) returns, for state-space model sys, an orthogonal matrix U such that (U*A*U',U*B,C*U') is a Kalman decomposition of (A,B,C)

% figure
% subplot(211);
% step(sys);
% title('Resposta a Degrau em Malha Aberta');
% subplot(212);
% step(sys_c);
% title('Resposta a Degrau em Malha Fechada');
