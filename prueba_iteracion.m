
dist_min=-10;
for p=1:1:200

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
C=sys_d.c; %Invariante

%Agrego un integrador para trabajar en lazo cerrado
%Amplio el sistema

AA=[A,zeros(4,1);-C*A, 1];
BB=[B;-C*B];
CC=[C 0];

%Verifico controlabilidad
M_c=[BB AA*BB AA^2*BB AA^3*BB AA^4*BB AA^5*BB];

%Dise�o con LQR
QQ=1*diag([1/10 1 1/0.1 1/0.2 1]);    RR=1e2;
QQ=1*diag([1/10 1 1/0.1 1/0.2 .001]);    RR=1e6;
QQ=1*diag([1/10 1 1/0.1 1/0.2 .01]);    RR=1e7; %este anda muy bien

[KK,S,polos_LC]=dlqr(AA,BB,QQ,RR);
%KK=[K -Ki]
K1=KK(1:4);
Ki1=-KK(5);

%Segundo controlador:
m=.1*10;

A_tc2=[0 1 0 0;0 -Fricc/M -m*g/M 0; 0 0 0 1; 0 -Fricc/(l*M) -g*(m+M)/(l*M) 0];
B_tc2=[0; 1/M; 0; 1/(l*M)];
C_tc2=[1 0 0 0]; 
D_tc2=0;

sys2=ss(A_tc2,B_tc2,C_tc2,D_tc2);
sys_d2=c2d(sys2,Ts,'zoh');

A2=sys_d2.a;
B2=sys_d2.b;
C2=sys_d2.c; %Invariante

AA2=[A2,zeros(4,1);-C2*A2, 1];
BB2=[B2;-C2*B2];
CC2=[C2 0];

QQ2=1*diag([10*rand(1)*1/10 10*rand(1)*1 10*rand(1)*1/0.1 10*rand(1)*1/0.2 rand(1)*.01]);    RR2=1e7;
Qb=QQ2;

[KK2,S,polos_LC2]=dlqr(AA2,BB2,QQ2,RR2);
%KK=[K -Ki]
K2=KK2(1:4);
Ki2=-KK2(5);


%Simulaci�n del control:

T=20;
T_switch=T;
deltat=10^-4;
Kmax=T/Ts;
pasos=round(T/deltat);
t=0:deltat:(T+T);

m=.1;
ta=0:deltat:(T+0.3*T-deltat);
ref=5*square(2*pi*ta/(2*T_switch)-pi)+5;
m1_var=(m/2)*square(2*pi*ta/(2*T_switch)-pi)+m/2;
m2_var=10*(m/2)*square(2*pi*ta/(2*T_switch))+10*m/2;
m_var=m1_var+m2_var;

Ci=[0 0 pi 0 0];

x=zeros(5,pasos);
x(1,1)=Ci(1)+10;
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
    if m_var(z)<0.5
        K=K1;
        Ki=Ki1;
    else
        K=K2;
        Ki=Ki2;
    end
    u=-K(1:4)*(x_k(1:4)-xOP)+Ki*v_k; %Sin observador
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

dist_max=min(x(1,:));
if dist_max>dist_min
    Qopt2=[Qb(1,1) Qb(2,2), Qb(3,3), Qb(4,4), Qb(5,5)];
    dist_min=dist_max
end
end
Qopt2