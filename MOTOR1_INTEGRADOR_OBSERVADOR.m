clc, clear all, close all;

%Motor con carga
%Variables de estado: x1=ia, x2=wr, x3=titat

%{
-------------------------------------------------------------------------
                    Comentarios/conclusiones/dudas

-------------------------------------------------------------------------
%}

Laa=366e-6;
J=5e-9;
Ra=55.6;
Bm=0;
Ki=7.49e-3;
Km=7.53e-3;

Ts=5e-5; %Tiempo de muestreo

Tl=1.15e-3; %Solo para la referencia de pi/2

A=[-Ra/Laa -Km/Laa 0; Ki/J -Bm/J 0; 0 1 0];
B=[1/Laa; 0; 0];
C=[0 0 1];
D=[0];

sys=ss(A,B,C,D);
sys_d=c2d(sys,Ts,'zoh');

A=sys_d.a;
B=sys_d.b;
C=sys_d.c;

%Agrego un integrador para trabajar a lazo cerrado
%Amplio el sistema

AA=[A,zeros(3,1);-C*A, 1];
BB=[B;-C*B];
CC=[C 0];

%Verifico controlabilidad
M=[BB AA*BB AA^2*BB AA^3*BB AA^4*BB];
rank(M) %=4, n=4 -> es controlable

%Diseño con LQR
QQ=1*diag([1 1/1000 1 10]);    RR=1e2;

KK=dlqr(AA,BB,QQ,RR);
%KK=[K -Ki]
eig(AA-BB*KK) %polos de lazo cerrado
K=KK(1:3);
Ki=-KK(4);

%Calculo del observador
Q0=[C; C*A; C*A^2];
rank(Q0) %=3, m=3 -> es observable

C_o=B';
A_o=A';
B_o=C';

Q_o=1*diag([1 1 1]);    R_o=1;
K_o=dlqr(A_o,B_o,Q_o,R_o);

%Simulación del control:

T=2;
T_switch=0.5;
deltat=1e-5;
Kmax=T/Ts;
pasos=round(T/deltat);
t=0:deltat:(T+T);

ta=0:deltat:(T+0.3*T-deltat);
ref=(pi/2)*square(2*pi*ta/(2*T_switch));
fTl=(Tl/2)*square(2*pi*ta/(2*T_switch))+Tl/2;

Ci=[0 0 0 0];

x=zeros(4,pasos);
x(1,1)=Ci(1);
x(2,1)=Ci(2);
x(3,1)=Ci(3);
x(4,1)=Ci(4);

x_hat(1,1)=Ci(1);
x_hat(2,1)=Ci(2);
x_hat(3,1)=Ci(3);

x_ts=x((1:3),1);
v_ts=x(4,1);
ua(1)=0;
z=1;

for i=1:1:Kmax+1
    x_k=x_ts;
    v_k=v_ts;
    %u=-K(1:3)*x_k(1:3)+Ki*v_k; %Sin observador
    u=-K(1:3)*x_hat(1:3)+Ki*v_k; %Con observador
    
    ua=[ua u*ones(1,round(Ts/deltat))];
    
    ys=C*x(1:3,z);
    for j=1:1:Ts/deltat 
        x1_p=-Ra*x(1,z)/Laa-Km*x(2,z)/Laa+u/Laa;
        x2_p=Ki*x(1,z)/J-Bm*x(2,z)/J-fTl(z)/J;
        x3_p=x(2,z);
        x_p_actual=[x1_p; x2_p; x3_p];
        
        x((1:3),z+1)=x((1:3),z)+deltat*x_p_actual;
        z=z+1;
    end
    yhat=C*x_hat;
    e=ys-yhat;
    x_hat=A*x_hat+B*u+K_o'*e;
    v_ts=v_ts+ref(z)-C*x_ts;
    x_ts=x((1:3),z);
end

%%
figure(1)
subplot(2,2,1)
hold on;
grid on;
plot(t(1:length(ua)),ua,'r');
xlim([0 T]);
title('Acción de control');
xlabel('Tiempo');
ylabel('Voltaje');
subplot(2,2,2)
hold on;
grid on;
plot(t(1:length(x(1,:))),x(1,:),'r');
xlim([0 T]);
title('Corriente');
xlabel('Tiempo');
subplot(2,2,[3,4])
hold on;
grid on;
plot(t(1:length(x(3,:))),x(3,:),'r'); 
xlim([0 T]);
plot(t(1:length(ref)),ref,'k');
legend({'Salida','Referencia'},'Location','southeast');
title('Salida del sistema');
xlabel('Tiempo');
ylabel('Ángulo');
%%
