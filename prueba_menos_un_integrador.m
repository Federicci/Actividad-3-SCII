clc, clear all, close all;

%Pendulo en el equilibrio estable

%{
-------------------------------------------------------------------------
                    Comentarios/conclusiones/dudas

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

%Agrego 1 integrador para trabajar en lazo cerrado, se pueden medir
%desplazamiento y angulo
%Amplio el sistema

AA=[A,zeros(4,1);-C(1,:)*A, eye(1)];
BB=[B;-C(1,:)*B];

%Verifico controlabilidad
M_c=[BB AA*BB AA^2*BB AA^3*BB AA^4*BB AA^5*BB];
rank(M_c) %=5, n=5 -> es controlable

%Diseño con LQR del primer controlador por variacion paramétrica de la masa
QQ=1*diag([1/15 1 1/0.1 1/0.2 .01]);    RR=1e7;

KK=dlqr(AA,BB,QQ,RR);
%KK=[K -Ki]
eig(AA-BB*KK) %polos de lazo cerrado
K1=KK(1:4);
Ki1=-KK(5);

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


AA2=[A2,zeros(4,1);-C2(1,:)*A2, eye(1)];
BB2=[B2;-C2(1,:)*B2];

%Diseño con LQR del segundo controlador
QQ2=1*diag([1/10 1/10 1/0.1 1/0.2 1]);    RR2=1e9;

KK2=dlqr(AA2,BB2,QQ2,RR2);
%KK=[K -Ki]
eig(AA2-BB2*KK2) %polos de lazo cerrado
K2=KK2(1:4);
Ki2=-KK2(5);

%Calculo del primer observador, para el primer caso de masa
Q0=[C; C*A; C*A^2; C*A^3];
rank(Q0) %=4, m=4 -> es observable

C_o=B';
A_o=A';
B_o=C';

Q_o=1*diag([1 10 1 10]);    R_o=[1e4 0; 0 1e4];
K_o1=dlqr(A_o,B_o,Q_o,R_o);

%Segundo observador, para cuando cambia la masa
Q0=[C2; C2*A2; C2*A2^2; C2*A2^3];
rank(Q0) %=4, m=4 -> es observable

C_o2=B2';
A_o2=A2';
B_o2=C2';

Q_o2=1*diag([1 1 1 1]);    R_o2=[7e5 7e3; 7e3 7e5];
K_o2=dlqr(A_o2,B_o2,Q_o2,R_o2);

%Simulación del control:

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
ua(1)=0;
z=1;
xOP=[0; 0; pi; 0];

for i=1:1:Kmax+1
    x_ts=x((1:4),z);
    v_ts=v_ts+ref(z)-C(1,:)*x_ts;
    ys=C*x(1:4,z); %Salida de dos componentes
    
    %Dependiendo de la masa a mover, se elige un controlador y observador distinto
    if m_var(z)<0.5
        K=K1;
        Ki=Ki1;
        K_o=K_o1;
    else
        K=K2;
        Ki=Ki2;
        K_o=K_o2;
    end
    
    %u=-K(1:4)*(x_ts(1:4)-xOP)+Ki*v_ts; %Sin observador
    u=-K(1:4)*(x_hat(1:4)-xOP)+Ki*v_ts; %Con observador
    
    %Descomentar para introducir alinealidad al actuador
    %{
    %Alinealidad
    if abs(u)<0.5
        u=0;
    else
        u=sign(u)*(abs(u)-0.5);
    end
    ua=[ua (u+sign(u)*0.5)*ones(1,round(Ts/deltat))];
    %}
    
    %ua=[ua u*ones(1,round(Ts/deltat))]; %Acumulador de accion de control
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
    %Observador y actualización de estados discretizados
    yhat=C*x_hat;
    e=ys-yhat;
    x_hat=A*(x_hat-xOP)+B*u+K_o'*e;
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
title('Acción de control');
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
title('Ángulo vs velocidad angular');
xlabel('Ángulo');
ylabel('Velocidad angular');