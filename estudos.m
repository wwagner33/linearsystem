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
% polso = pole(sys);
% 
% %Teste de Observabilidade
% Obs = obsv(A,C);
% isobs = length(A)-rank(Obs); % 0 - Sistema eh observavel
% 
% %Teste de Controlabilidade
% [Abar,Bbar,Cbar,T,k] = ctrbf(A,B,C);

%Forma Aumentada
Al=[A zeros(length(A),1);-C 0];
Bl = [B;0];
Cl = [C 0];
El = [zeros(length(A),1);1];
disturbance = ones(length(Bl),1)*1.1;



%Polos desejados em sistema de Malha Aberta
sys_o = ss(Al,[Bl El],Cl,0);
cp=charpoly(Al);
poles=[-2+2j -2-2j -4+4j -4-4j];

%Realimentacao de Estados
Kl = place(Al,Bl,poles);

%Sistema em Malha Fechada
Ac = Al-Bl*Kl;
sys_c = ss(Ac,El,Cl,0);

figure
subplot(211);
step(sys_o);
title('Resposta a Degrau em Malha Aberta');
subplot(212);
step(sys_c);
title('Resposta a Degrau em Malha Fechada');
