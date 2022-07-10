% Work to Linear System Discipline
% PPGEE
% Federal University of Ceará
% Authors:
%     Patrícia de Sousa Paula
%     Wellington Wagner F. Sarmento

clear all; close all; clc;


% extraido de gs = n/d
d = [1 1 2 -2]; %denominador - polinomio caracteristico
n = [1 1 1]; %numerador - polinomio minimo (?)

% converte para representacao state-space
[A, B, C, D] = tf2ss(n, d);
sys = tf(n,d);

cp = charpoly(A);%polinomio caracteristico do sistema - retorna os coeficientes

CM = ctrb(A,B); %calcula Matriz de Controlabilidade

% define planta aumentada
tempAa = [A;-C];
Aa = [tempAa zeros(length(tempAa),1)];
Ba = [B;0];
Ca = [C 0];

%calcula o K aumentado
dp = [-2+2j,-2-2j,-4+4j,-4-4j]; %polo desejados

Ka = place(Aa,Ba,dp);


%determina K e Kz
K=Ka(1:3);
Kz=Ka(4);


%projeta estimador de estado

%teste de observabilidade
Ob = obsv(A,C);
unob = length(A)-rank(Ob); %resulta em 0 (zero), ou seja, nao possui estados nao observaveis

CM_dual_bar = inv([1 0 cp(2);0 1 0;0 0 1]);

q=[-2+2j -2-2j -8]; %A-L'*C

L = place(A',C',q).'; %calcula o ganho de Pole Placement usando a formula de Ackermann

ALC = A-L*C;

