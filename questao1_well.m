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

cp = charpoly(A);%polinomio caracteristico do sistema

CM = ctrb(A,B); %calcula Matriz de Controlabilidade

% if (eq(rank(CM),length(A)) == 1) %testa controlabilidade
%     "Sistema é Controlável"
% else
%     "Sistema não é controlável"
% end

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

%definicao do disturbio na entrada
d_i=1;

% 
% %projeta estimador de estado
% 
% q = -8;
% 
% L = place(A',C',q);
