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
C_tc=[0 0 0 1; 1 0 0 0];
D_tc=0;
%x1=alpha x2=phi x3=phi_p x4=h

Ts=1e-3; %Tiempo de muestreo
Ts=1e-1; %Tiempo de muestreo
sys=ss(A_tc,B_tc,C_tc,D_tc);
sys_d=c2d(sys,Ts,'zoh');

A=sys_d.a;
B=sys_d.b;
C=sys_d.c;

%Agrego un integrador para trabajar a lazo cerrado:
%Amplio el sistema

AA=[A,zeros(4,1);-C(1,:)*A, eye(1)];
BB=[B;-C(1,:)*B];
CC=[C(1,:) zeros(1,1)];

%Verifico controlabilidad
M=[BB AA*BB AA^2*BB AA^3*BB AA^4*BB  AA^5*BB];
rank(M) %=5, n=5 -> es controlable

%Diseño con LQR
QQ=1e-15*diag([1 1 1/10 1/100000 .01]);    RR=1;
QQ=1e-15*diag([1 1 1/10 1/100000 500]);    RR=1;

KK=dlqr(AA,BB,QQ,RR);
%KK=[K -Ki]
eig(AA-BB*KK) %polos de lazo cerrado
K=KK(1:4);
Ki=-KK(5);

%Calculo del observador
Q0=[C; C*A; C*A^2; C*A^3];
rank(Q0) %=4, m=4 -> es observable

C_o=B';
A_o=A';
B_o=C';

Q_o=1*diag([1 1 1 1]);    R_o=[1 0; 0 1];
K_o=dlqr(A_o,B_o,Q_o,R_o);

%Simulación del control:

T=500;
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
z=1;

for i=1:1:Kmax+1
    x_k=x_ts;
    v_k=v_ts;
    
    %u=-K*x_k+Ki*v_k; %Sistema sin observador
    u=-K(1:4)*x_hat(1:4)+Ki*v_k; %Con observador
    uu=u;
    
    %Alinealidad
    Alin=0.001;
    if abs(u)<Alin
        u=0;
    else
        u=sign(u)*(abs(u)-Alin);
    end
    
    ys=C*x(1:4,z); %Salida de dos componentes
    for j=1:1:Ts/deltat 
        ua(z)=uu;
        x_p_actual=A_tc*x(1:4,z)+B_tc*u;
        x((1:4),z+1)=x((1:4),z)+deltat*x_p_actual;
        
        z=z+1;
    end
    yhat=C*x_hat;
    e=ys-yhat;
    x_hat=A*x_hat+B*u+K_o'*e;
    v_ts=v_ts+ref(z)-C(1,:)*x_ts;
    x_ts=x((1:4),z);
end
x(4,:)=x(4,:)+1000*ones(1,length(x(4,:)));
%%
figure(1)
subplot(3,2,1);
grid on;
hold on;
plot(t(1:length(x(4,:))),x(4,:),'color','r');
plot(t(1:length(ref)),ref+1000,'k');
title('Altura');
xlabel('Tiempo');
xlim([0 T]);
ylim([0 2000]);
legend({'Salida','Referencia'},'Location','southeast');

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
zm=ones(2,length(ua));
zm(1,:)=zm(1,:)*Alin;
zm(2,:)=zm(2,:)*-Alin;
plot(t(1:length(ua)),zm,'k');
title('Acción de control');
xlabel('Tiempo');
xlim([0 T]);
ylim([-Alin-.3 Alin+.3]);
legend({'Acción de control','Zona muerta'},'Location','northeast');