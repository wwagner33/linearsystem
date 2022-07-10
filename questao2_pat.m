clear all; close all;  clc; 

s = sym('s');

A = [1 2 0; -1 -1 -1; 0 2 -1];
B = [0;0;2];
C = [0 1 0; 0 0 1];
D = 0;

% sistema aberto
sysop = ss(A, B, C, D);
[n,d] = tfdata(sysop);

x0 = [1; -1; 1];

% poles = eig(A);

% matrix de controlabilidade
Co = ctrb(A,B);
% num estados nao controlaveis
nc = length(A) - rank(Co); % 0 -> controlavel

% matriz de observabilidade
Obs = obsv(A,C);
no = length(A) - rank(Obs); % 0 -> observável

% polinomio caracteristico: det(si - A)
si = [s 0 0;0 s 0;0 0 s];
polc = det(si - A);

% polos desejados
newpoles = [-1.5+i -1.5-i -2];
K = place(A,B,newpoles);

% malha fechada com realim estado
Acl = A - B*K;
Ecl = eig(Acl);
syscl = ss(Acl, B, C, D);

% observador
Pe = [-5+5*j -5-5*j -8];
L = place(A',C',Pe);
L = L';
Alc = A - L * C;

N = -1/(C*inv(A-B*K)*B);
%Q = 

% sistema fechado com realimentacao
%syscl = ss(Acl,Ba,Ca,D);

% polinomio caracteristico com polos desejados (aumentado)
%polck = (s+2+2*i)*(s+2-2*i)*(s+4+4*i)*(s+4-4*i);
% calculando Kbarrado
%Kbar = sym2poly(polck)-[0 sym2poly(polc)]; % polc nao tem s4, por isso coloco 0*s4
%Kbar = Kbar(2:5);

%[Aabar,Babar,Cabar,P,k] = ctrbf(Aa,Ba,Ca);
%Cobar = [Babar Aa*Babar Aa^2*Babar Aa^3*Babar];
%Ka = Kbar*Cobar*inv(Coa);
%K = Ka(1:3);
%Kz = Ka(4);

figure;
subplot(211);
step(sysop);
title('Resposta Malha Aberta');
subplot(212);
step(syscl);
title('Resposta Malha Fechada');
