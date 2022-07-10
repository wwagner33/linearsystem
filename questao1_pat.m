clear all; close all;  clc; 
s = sym('s');

gs = (s^2+s+1)/(s^4+s^3+2*s^2-2);

% extraido de gs
a = [1 1 2 -2];
b = [1 1 1];

% converte para representacao state-space
[A,B,C,D] = tf2ss(b,a);

% sistema aberto
sysop = ss(A,B,C,D);

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

% polos desejados - eq estado aumentado
Aa = [A zeros(length(A),1);-C 0];
Ba = [B;0];
Ca = [C 0];
Ea = [zeros(length(A),1);1];
Coa = ctrb(Aa,Ba); % = [Ba Aa*Ba Aa^2*Ba Aa^3*Ba];
Obsa = obsv(Aa,Ca);

% K com polos desejados
newpoles = [-2+2*i -2-2*i -4+4*i -4-4*i];
Ka = place(Aa,Ba,newpoles);
Kz = Ka(4);
K = Ka(1:3);

% malha fechada com realim estado
Acl = Aa - Ba*Ka;
Ecl = eig(Acl);

% sistema fechado com realimentacao
syscl = ss(Acl,Ba,Ca,D);

% estimador
Pe = [-8 -8 -8];
L = acker(A',C',Pe);
L = L';
Alc = A - L * C;

figure(1);
subplot(211);
step(sysop);
title('Resposta Malha Aberta');
subplot(212);
step(syscl);
title('Resposta Malha Fechada');

%figure(2);
%hold on;
%grid;
%axis;
%plot(eig(A),'o','color','b');
%plot(eig(Acl),'*','color','r');
%title('Polos Malha Aberta [o] e Fechada [*]');
