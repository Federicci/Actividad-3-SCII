clc, clear all, close all;

%{
-------------------------------------------------------------------------
                    Comentarios/conclusiones/dudas
-------------------------------------------------------------------------
%}

w=9;
a=0.07;
b=5;
c=150;

A_tc=[-a a 0 0; 0 0 1 0; w^2 -w^2 0 0; c 0 0 0];
B_tc=[0; 0; b*w^2; 0];
C_tc=[0 0 0 1];
D_tc=0;
%x1=alpha x2=phi x3=phi_p x4=h

Ts=1e-2; %Tiempo de muestreo
Ts=1e-3; %Tiempo de muestreo
sys=ss(A_tc,B_tc,C_tc,D_tc);
sys_d=c2d(sys,Ts,'zoh');

A=sys_d.a;
B=sys_d.b;
C=sys_d.c; %Invariante

%Agrego un integrador para trabajar a lazo cerrado:
%Amplio el sistema

AA=[A,zeros(4,1);-C*A, 1];
BB=[B;-C*B];
CC=[C 0];

%Verifico controlabilidad
M=[BB AA*BB AA^2*BB AA^3*BB AA^4*BB  AA^5*BB];
rank(M) %=5, n=5 -> es controlable

%Diseño con LQR
QQ=1*diag([1/100 1/100 1/100 1/1000 .001]);    RR=1e10;
QQ=1*diag([1/1000 1/1000 1/1000 1/10000 .0001]);    RR=1e10;
QQ=1*diag([1/1000 1/100 1/100 1/10000 .001]);    RR=1e8;
QQ=1*diag([1 1/3 1/10 1/10000 .001]);    RR=1e15;
QQ=1*diag([1 1/3 1/10 1/10000 .001]);    RR=1e15;
QQ=1e-15*diag([1 1 1/10 1/100000 .01]);    RR=1;
%QQ=1*diag([1 1/3 1/10 1/10000 1]);    RR=1e17;

[KK,S,polos_LC]=dlqr(AA,BB,QQ,RR);
%KK=[K -Ki]
polos_LC
eig(AA-BB*KK)
K=KK(1:4);
Ki=-KK(5);

%Calculo del observador
Q0=[C; C*A; C*A^2; C*A^3];
rank(Q0) %=4, m=4 -> es observable

C_o=B';
A_o=A';
B_o=C';

Q_o=1*diag([1 1 1 1]);    R_o=1;
[K_o,S_o,polos_o]=dlqr(A_o,B_o,Q_o,R_o);
polos_o

%Simulación del control:

T=130;
deltat=1e-4;
Kmax=T/Ts;
pasos=round(T/deltat);
t=0:deltat:(T+T);

ta=0:deltat:(T+0.3*T-deltat);
ref=-100*ones(1,length(ta));

Ci=[0 0 0 500 0];

x=zeros(5,pasos);
x(1,1)=Ci(1);
x(2,1)=Ci(2);
x(3,1)=Ci(3);
x(4,1)=Ci(4);

x_hat(1,1)=Ci(1);
x_hat(2,1)=Ci(2);
x_hat(3,1)=Ci(3);
x_hat(4,1)=Ci(4);

x_ts=x((1:4),1);
v_ts=x(5,1);
ua(1)=0;
z=1;

error1=0;

for i=1:1:Kmax+1
    x_k=x_ts;
    v_k=v_ts;
    %u=-K*x_k+Ki*v_k; %Sistema sin observador
    %u=-K(1)*x_hat(1)-K(2:4)*x_k(2:4)+Ki*v_k; %Solo alpha observado
    u=-K(1:4)*x_hat(1:4)+Ki*v_k; %Todo observado
    if abs(u)<0.05
        u=0;
    else
        u=sign(u)*(abs(u)-0.05);
    end
    ua=[ua (u+sign(u)*0.05)*ones(1,round(Ts/deltat))];
    ys=C*x(1:4,z);
    for j=1:1:Ts/deltat 
        x_p_actual=A_tc*x(1:4,z)+B_tc*u;
        x((1:4),z+1)=x((1:4),z)+deltat*x_p_actual;
        
        z=z+1;
    end
    error1=[error1 (x(1,z)-x_hat(1))*ones(1,round(Ts/deltat))];
    yhat=C*x_hat;
    e=ys-yhat;
    x_hat=A*x_hat+B*u+K_o'*e;
    v_ts=v_ts+ref(z)-C*x_ts;
    x_ts=x((1:4),z);
end
%%
figure(1)
subplot(3,2,1);
grid on;
hold on;
plot(t(1:length(x(4,:))),x(4,:),'color','r');
plot(t(1:length(ref)),ref,'k');
title('Altura');
xlabel('Tiempo');
xlim([0 T]);

subplot(3,2,2);
hold on;
grid on;
plot(t(1:length(x(1,:))),x(1,:),'color','r');
title('Alpha');
xlabel('Tiempo');
xlim([0 T]);

subplot(3,2,3);
hold on;
grid on;
plot(t(1:length(x(2,:))),x(2,:),'color','r');
title('Phi');
xlabel('Tiempo');
xlim([0 T]);

subplot(3,2,4);
hold on;
grid on;
plot(t(1:length(x(3,:))),x(3,:),'color','r');
title('Phi punto');
xlabel('Tiempo');
xlim([0 T]);

subplot(3,2,[5,6]);
hold on;
grid on;
plot(t(1:length(ua)),ua,'color','r');
title('Acción de control');
xlabel('Tiempo');
xlim([0 T]);

figure
hold on;
grid on;
plot(t(1:length(error1)),error1);
xlim([0 T]);
title('Error de observación');
xlabel('Tiempo');
ylabel('Alpha real - Alpha observado');