%...ETN-1034...II/2016...%
%...Laboratorio N°1...Control por Realimentación de Estados...%
close all 
clear
clc
%...Espacio de Estados Contínuo...%
A=[0 1;0 0];
B=[0 ;336.34];                            %%22.979 4.2122  14.4163 10.1432
C=[1 0];
D=0;
 
Gp=ss(A,B,C,D);
T=0.05;             %...Periodo de muestreo...%

%%Especificaciónes de Desempeño
Mp=6;               %...Máximo sobrepaso...%
ts=2;               %...tiempo de asentamiento...%
eps=sqrt(((log(Mp/100))^2)/((pi)^2+((log(Mp/100))^2)));
sigma=4/ts;
wn=sigma/eps;
wd=wn*sqrt(1-eps^2);
p1=-sigma+wd*1i;
p2=-sigma-wd*1i;
pc=[p1 p2];

%...Espacio de Estados discreto...%
Gpd=c2d(Gp,T,'zoh');

%%Regulador matriz de Ganancia K
pd=exp(T*pc);
Kd=acker(Gpd.a,Gpd.b,pd);

%%Seguidor Precompensación N
[n,d]=ss2tf(Gpd.a-Gpd.b*Kd,Gpd.b,Gpd.c,Gpd.d);
Nd=(d(1)+d(2)+d(3))/(n(1)+n(2)+n(3));

%%Observador de Orden Completo
r=abs(pd(1));
ang=angle(pd(1));
t=-T/(log(r));
t_obs=t/4;
r_obs=exp(-T/t_obs);
ang_obs=sqrt((log(r_obs)/eps)^2-(log(r_obs))^2);
pdo=[r_obs*cos(ang_obs)+r_obs*sin(ang_obs)*1i; r_obs*cos(ang_obs)-r_obs*sin(ang_obs)*1i];

Gd=acker(Gpd.a',Gpd.c',pdo)';

%%Observador de Orden Reducido
pdor=exp(-T/t_obs);
Aaa=Gpd.a(1,1); Aab=Gpd.a(1,2);
Aba=Gpd.a(2,1); Abb=Gpd.a(2,2);

Gdr=acker(Abb',Aab',pdor)';

% orden completo
Gpdlc=ss(Gpd.a-Gpd.b*Kd,Nd*Gpd.b,Gpd.c,Gpd.d,T);
% observador orden completo
Gpdlco=ss([Gpd.a-Gpd.b*Kd Gpd.b*Kd;[0 0;0 0] Gpd.a-Gd*Gpd.c],[Nd*Gpd.b;0;0],[Gpd.c 0 0],Gpd.d,T);

%******...REALIMENTACION CON ACCION INTEGRAL (PAG.462 EN ADELANTE, K. OGATA. CON OBSERVADOR)...***%%%   
pd=[pd 0];
GG=[Gpd.a(1,:) Gpd.b(1);Gpd.a(2,:) Gpd.b(2);0 0 0];
HH=[0;0;1];
%Kac=[0 0 1]*inv([HH GG*HH ((GG)^2)*HH])*(GG^4)   %***CONDICION %"PERFECTA"
Kac=[0 0 1]*inv([HH GG*HH ((GG)^2)*HH])*((GG)^3 - 1.7984*(GG)^2 + 0.8187*GG);   %%%%***CONDICION DESEADA***%%%
KO=(Kac+[0 0 1])*inv([Gpd.a-eye(2) Gpd.b;Gpd.c*Gpd.a Gpd.c*Gpd.b])      %MATRIZ [K1 K2 Ki];

%Funcion de transferencia con observador de orden completo
syms z
detd=vpa(det(z*eye(4)-Gpdlco.a),5);
numd=vpa(z*eye(4)-Gpdlco.a);
numd=vpa(adjoint(numd),5);
numd=vpa(Gpdlco.c*numd*Gpdlco.b,5);
tfz_comp = vpa((numd/detd),5)

%estabilidad (ECUACIÓN DE LYAPUNOV)
%Directamente
syms p11 p12 p13 p14 p21 p22 p23 p24 p31 p32 p33 p34 p41 p42 p43 p44
p=[p11 p12 p13 p14;p21 p22 p23 p24;p31 p32 p33 p34;p41 p42 p43 p44];
pp=vpa((Gpdlco.a'*p*Gpdlco.a)-p,4);
sol1=solve(pp==-eye(4));
sol = structfun(@double,sol1);
P=vpa([sol(1) sol(2) sol(3) sol(4);sol(5) sol(6) sol(7) sol(8);sol(9) sol(10) sol(11) sol(12);sol(13) sol(14) sol(15) sol(16)],5)

%Mediante la serie
P1=eye(4)*eye(4)*eye(4);
for i=1:500
P1=vpa(P1+((Gpdlco.a')^i)*eye(4)*(Gpdlco.a)^i,5);
end

[zz,pol]=ss2zp(Gpdlco.a,Gpdlco.b,Gpdlco.c,Gpdlco.d);

%Observador completo opcional
syms o1 o2
o=[o1;o2];
Gobs=det(z*eye(2)-Gpd.a+o*Gpd.c);
[qq,rr]=ss2tf(Gpdlco.a,Gpdlco.b,Gpdlco.c,Gpdlco.d);
tfz_completo = tf(qq,rr,0.05)
%step(tfz_completo);
%grid on;

