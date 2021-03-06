
%condicao inicial X(0)
X0 =[1;-1;1];

%sistema
A=[1 2 0;-1 -1 -1;0 2 -1];
B=[0;0;2];
C=[0 1 0;0 0 1];
D=[0;0];

%Controlador LQR

Q=[1 0 0;0 1 0;0 0 1]; %penaliza erro

R= 1;%penaliza atuador

K=lqr(A,B,Q,R);

%sistema em malha fechada

sys = ss((A-B*K),B,C,D);

%execucao do sistema com condicao inicial X0

[Y,T,X] = initial(sys,X0);

%cria os graficos

if ~exist('h1','var')
    h1= figure('Position',[100 670 670 238]);
    title('Resposta as Condicoes Iniciais','FontSize',14);
    h1.ToolBar = 'none';
end

if ~exist('h2','var')
    h2= figure('Position',[100 390 670 234]);
    title('Esforco do Atuador','FontSize',14);
    h2.ToolBar = 'none';
end

if ~exist('h3','var')
    h3= figure('Position',[100 80 670 236]);
    title('Mapa de Polos e Zeros','FontSize',14);
    h2.ToolBar = 'none';
end

% plot das respostas
set(0,'currentfigure',h1);
hold all
p1 = plot(T,Y(:,1),'LineWidth',4);

%plot do esforco do atuador
set(0,'currentfigure',h2);
hold all
p2 = plot(T,(-K*X'),'LineWidth',4);

%plot dos polos e zeros
set(0,'currentfigure',h3);
hold all
pzmap(sys);



