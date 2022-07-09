clear all; close all; clc;

s=tf('s');
% extraido de gs
d = [1 1 2 -2]; %denominador
n = [1 1 1]; %numerador

% atribuindo um valor para r
r=1;

% converte para representacao state-space
[A, B, C, D] = tf2ss(n, d);


sys = tf(n,d);

%Forma Aumentada
Aa=[A zeros(length(A),1);-C 0];
Ba = [B;0];
Ca = [C 0];
Ea = [zeros(length(A),1);1];
% disturbance = ones(length(Bl),1)*1.1;



%Polos desejados em sistema de Malha Aberta
sys_o = ss(Aa,[Ba Ea],Ca,0);
poles_d=[-2+2j -2-2j -4+4j -4-4j];

%Realimentacao de Estados
Ka = place(Aa,Ba,poles_d);

poles=pole(sys);
K=place(A,B,poles);

%Sistema em Malha Fechada
Ac = Aa-Ba*Ka;
sys_c = ss(Ac,Ea,Ca,0);

figure
subplot(211);
step(sys);
title('Resposta a Degrau em Malha Aberta');
subplot(212);
step(sys_c);
title('Resposta a Degrau em Malha Fechada');
