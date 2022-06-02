clc, clear all, close all;

%Pendulo en el equilibrio estable

%{
-------------------------------------------------------------------------
                    Comentarios/conclusiones/dudas
Con un solo controlador se complica el cambio de masa
Igualmente el objetivo del trabajo es trabajar solo con observador, este
fue un código de prueba
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
C_tc=[1 0 0 0]; 
D_tc=0;

%[delta delta_p phi phi_p]

Ts=1e-3; %Tiempo de muestreo

sys=ss(A_tc,B_tc,C_tc,D_tc);
sys_d=c2d(sys,Ts,'zoh');

A=sys_d.a;
B=sys_d.b;
C=sys_d.c;

%Agrego un integrador para trabajar en lazo cerrado
%Amplio el sistema

AA=[A,zeros(4,1);-C*A, 1];
BB=[B;-C*B];
CC=[C 0];

%Verifico controlabilidad
M_c=[BB AA*BB AA^2*BB AA^3*BB AA^4*BB AA^5*BB];
rank(M_c) %=5, n=5 -> es controlable

%Diseño con LQR
QQ=1*diag([1/10 1 1/0.1 1/0.2 .01]);    RR=1e7;

KK=dlqr(AA,BB,QQ,RR);
%KK=[K -Ki]
eig(AA-BB*KK) %polos de lazo cerrado
K=KK(1:4);
Ki=-KK(5);

%Simulación del control:

T=70;
T_switch=T/2;
deltat=10^-4;
Kmax=T/Ts;
pasos=round(T/deltat);
t=0:deltat:(T+T);

ta=0:deltat:(T+0.3*T-deltat);
ref=5*square(2*pi*ta/(2*T_switch))+5;
m1_var=(m/2)*square(2*pi*ta/(2*T_switch))+m/2;
m2_var=10*(m/2)*square(2*pi*ta/(2*T_switch)-pi)+10*m/2;
m_var=m1_var+m2_var;

Ci=[0 0 pi 0 0];

x=zeros(5,pasos);
x(1,1)=Ci(1);
x(2,1)=Ci(2);
x(3,1)=Ci(3);
x(4,1)=Ci(4);
x(5,1)=Ci(5);

x_ts=x((1:4),1);
v_ts=x(5,1);
ua(1)=0;
z=1;
xOP=[0; 0; pi; 0];

for i=1:1:Kmax+1
    x_k=x_ts;
    v_k=v_ts;
    u=-K(1:4)*(x_k(1:4)-xOP)+Ki*v_k; %Sin observador
    ua=[ua u*ones(1,round(Ts/deltat))];
    ys=C*x(1:4,z);
    for j=1:1:Ts/deltat 
        x_actual=x((1:4),z)-xOP;
        x1_p=x_actual(2);
        x2_p=-Fricc*x_actual(2)/M-m_var(z)*g*x_actual(3)/M+u/M;
        x3_p=x_actual(4);
        x4_p=-Fricc*x_actual(2)/(l*M)-g*(m_var(z)+M)*x_actual(3)/(l*M)+u/(l*M);
        x_p_actual=[x1_p; x2_p; x3_p; x4_p];
        
        x((1:4),z+1)=x((1:4),z)+deltat*x_p_actual;
        z=z+1;
    end
    v_ts=v_ts+ref(z)-C*x_ts;
    x_ts=x((1:4),z);
end

%%
figure
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
title('Acción de control');
xlabel('Tiempo');

%{
figure
hold on;
grid on;
plot(t(1:length(error)),error);
xlim([0 T]);
title('Error de observación');
xlabel('Tiempo');
ylabel('Corriente real - Corriente observada');
%}
%%