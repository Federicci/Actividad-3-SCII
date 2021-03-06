clc, clear all, close all;

%Pendulo en el equilibrio estable

%{
-------------------------------------------------------------------------
                    Comentarios/conclusiones/dudas
Ademas de usar dos controladores, ademas tengo 2 integradores, se miden 2
salidas
En este c?digo tambien se analiza el caso con alinealidad
-------------------------------------------------------------------------
%}

m=.1;
Fricc=0.1; 
l=2.6;
g=9.8;
M=.5;

%Linealizado en el equilibrio estable:
A_tc=[0 1 0 0;0 -Fricc/M -m*g/M 0; 0 0 0 1; 0 -Fricc/(l*M) -g*(m+M)/(l*M) 0];
B_tc=[0; 1/M; 0; 1/(l*M)];
C_tc=[1 0 0 0; 0 0 1 0]; 
D_tc=[0; 0];

%[delta delta_p phi phi_p]

Ts=1e-3; %Tiempo de muestreo

sys=ss(A_tc,B_tc,C_tc,D_tc);
sys_d=c2d(sys,Ts,'zoh');

A=sys_d.a;
B=sys_d.b;
C=sys_d.c; 

%Agrego dos integradores para trabajar en lazo cerrado, se pueden medir
%desplazamiento y angulo
%Amplio el sistema

AA=[A,zeros(4,2);-C*A, eye(2)];
BB=[B;-C*B];

%Verifico controlabilidad
M_c=[BB AA*BB AA^2*BB AA^3*BB AA^4*BB AA^5*BB AA^6*BB];
rank(M_c) %=6, n=6 -> es controlable

%Dise?o con LQR del primer controlador por variacion param?trica de la masa
QQ=1*diag([1/15 1 1/0.1 1/0.2 .01 .01]);    RR=1e7;

KK=dlqr(AA,BB,QQ,RR);
%KK=[K -Ki]
eig(AA-BB*KK) %polos de lazo cerrado
K1=KK(1:4);
Ki1=-KK(5);
Ki12=-KK(6);

%Segundo controlador para cuando cambie la masa:
m=.1*10;

A_tc2=[0 1 0 0;0 -Fricc/M -m*g/M 0; 0 0 0 1; 0 -Fricc/(l*M) -g*(m+M)/(l*M) 0];
B_tc2=[0; 1/M; 0; 1/(l*M)];
C_tc2=[1 0 0 0; 0 0 1 0]; 
D_tc2=[0; 0];

sys2=ss(A_tc2,B_tc2,C_tc2,D_tc2);
sys_d2=c2d(sys2,Ts,'zoh');

A2=sys_d2.a;
B2=sys_d2.b;
C2=sys_d2.c;

%Sigo trabajando con 2 integradores, se debe ampliar el sistema

AA2=[A2,zeros(4,2);-C2*A2, eye(2)];
BB2=[B2;-C2*B2];

%Dise?o con LQR del segundo controlador
QQ2=1*diag([1/10 1/10 1/0.1 1/0.2 .1 .1]);    RR2=1e10;

KK2=dlqr(AA2,BB2,QQ2,RR2);
%KK=[K -Ki]
eig(AA2-BB2*KK2) %polos de lazo cerrado
K2=KK2(1:4);
Ki2=-KK2(5);
Ki22=-KK2(6);

%Simulaci?n del control:

T=80;
T_switch=T/2;
%T_switch=T; %descomentar para simular solo cambio de masa
deltat=10^-4;
Kmax=T/Ts;
pasos=round(T/deltat);
t=0:deltat:(T+T);

m=.1;
ta=0:deltat:(T+0.3*T-deltat);
ref=5*square(2*pi*ta/(2*T_switch))+5;
%ref=0*5*square(2*pi*ta/(2*T_switch))+5-5; %descomentar para simular solo cambio de masa
m1_var=(m/2)*square(2*pi*ta/(2*T_switch))+m/2;
m2_var=10*(m/2)*square(2*pi*ta/(2*T_switch)-pi)+10*m/2;
%m2_var=10*(m/2)*square(2*pi*ta/(2*T_switch))+10*m/2; %descomentar para simular solo cambio de masa
m_var=m1_var+m2_var;
%m_var=0*m1_var+m2_var;  %descomentar para simular solo cambio de masa

Ci=[0 0 pi 0 0 0];

x=zeros(6,pasos);
x(1,1)=Ci(1);
%x(1,1)=Ci(1)+10; %descomentar para simular solo cambio de masa
x(2,1)=Ci(2);
x(3,1)=Ci(3);
x(4,1)=Ci(4);
x(5,1)=Ci(5);
x(6,1)=Ci(6);

x_hat(1,1)=Ci(1);
%x_hat(1,1)=Ci(1)+10; %descomentar para simular solo cambio de masa
x_hat(2,1)=Ci(2);
x_hat(3,1)=Ci(3);
x_hat(4,1)=Ci(4);

x_ts=x((1:4),1);
v_ts=x(5,1);
v_ts2=x(6,1);
z=1;
xOP=[0; 0; pi; 0];

for i=1:1:Kmax+1
    x_k=x_ts;
    v_k=v_ts;
    v_k2=v_ts2;
    
    %Dependiendo de la masa a mover, se elige un controlador distinto
    if m_var(z)<0.5
        K=K1;
        Ki=Ki1;
        Ki2=Ki12;
    else
        K=K2;
        Ki=Ki2;
        Ki2=Ki22;
    end
    
    u=-K(1:4)*(x_k(1:4)-xOP)+Ki*v_k; %Sin observador
    
    %Descomentar para introducir alinealidad al actuador
    %{
    %Alinealidad
    Alin=0.5;
    if abs(u)<Alin
        u=0;
    else
        u=sign(u)*(abs(u)-Alin);
    end
    
    ua=[ua (u+sign(u)*Alin)*ones(1,round(Ts/deltat))];
    %}
    
    ys=C*x(1:4,z); %Salida de dos componentes
    for j=1:1:Ts/deltat
        ua(z)=u;
        %Evolucion del sistema en un Ts
        x_actual=x((1:4),z)-xOP;
        x1_p=x_actual(2);
        x2_p=-Fricc*x_actual(2)/M-m_var(z)*g*x_actual(3)/M+u/M;
        x3_p=x_actual(4);
        x4_p=-Fricc*x_actual(2)/(l*M)-g*(m_var(z)+M)*x_actual(3)/(l*M)+u/(l*M);
        x_p_actual=[x1_p; x2_p; x3_p; x4_p];
        
        x((1:4),z+1)=x((1:4),z)+deltat*x_p_actual;
        z=z+1;
    end
    v_ts=v_ts+ref(z)-C(1,:)*x_ts;
    v_ts2=v_ts+pi-C(2,:)*x_ts;
    x_ts=x((1:4),z);
end

%Graficos
%%
figure(1)
subplot(2,2,1)
hold on;
grid on;
plot(t(1:length(x(1,:))),x(1,:),'r');
plot(t(1:length(ref)),ref,'k');
xlim([0 T]);
title('Distancia');
xlabel('Tiempo');
legend({'Salida','Referencia'},'Location','southeast');

subplot(2,2,2)
hold on;
grid on;
plot(t(1:length(x(3,:))),x(3,:),'r');
xlim([0 T]);
title('Angulo');
xlabel('Tiempo');

subplot(2,2,[3,4])
hold on;
grid on;
plot(t(1:length(ua)),ua,'r'); 
xlim([0 T]);
title('Acci?n de control');
xlabel('Tiempo');

%Planos de fase
figure(2)
subplot(2,1,1);
grid on;
hold on;
plot(x(1,1),x(2,1),'b');
f=-t(1:length(x(1,:))).^1.2/(T^1.2)+1;
f(end)=1;
C=zeros(length(x(1,:)),3);
C(:,3)=f;
scatter(x(1,:),x(2,:),3,C,'filled')
title('Distancia vs velocidad');
xlabel('Distancia');
ylabel('Velocidad');

subplot(2,1,2);
grid on;
hold on;
plot(x(3,1),x(4,1),'b');
f=-t(1:length(x(1,:))).^1.2/(T^1.2)+1;
f(end)=1;
C=zeros(length(x(1,:)),3);
C(:,3)=f;
scatter(x(3,:),x(4,:),3,C,'filled')
title('?ngulo vs velocidad angular');
xlabel('?ngulo');
ylabel('Velocidad angular');