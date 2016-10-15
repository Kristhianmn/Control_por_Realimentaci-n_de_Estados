%Kalman
clc;
clear all;
close all;
% Sistema
A=[0 1;-0.5 1];    B=[0;1];   C=[0.5 1];      D=0;
pd=[0.25+0.25*i 0.25-0.25*i];
T=0.05;

%%Regulador matriz de Ganancia K
Kd=acker(A,B,pd)
%%Seguidor Precompensación N
[n d]=ss2tf(A-B*Kd,B,C,D)                       %transformación de EE a FdT
Nd=(d(1)+d(2)+d(3))/(n(1)+n(2)+n(3))
NN=-1/(C*(inv(A-B*Kd))*B)
%%Observador de Orden Completo
r=abs(pd(1));
ang=angle(pd(1));
t=-T/(log(r));
t_obs=t/4;
r_obs=exp(-T/t_obs);
ang_obs=sqrt((log(r_obs)/eps)^2-(log(r_obs))^2);
pdo=[r_obs*cos(ang_obs)+r_obs*sin(ang_obs)*1i; r_obs*cos(ang_obs)-r_obs*sin(ang_obs)*1i];
Gd=acker(A',C',pdo)'

t=0:T:10;
ref=5*heaviside(t);
G=[0.043;-0.047];
% Covarianzas de Ruidos
V=0.05; % ruido de proceso 0.09
W=0.015; % ruido de medicion 0.025
% Condiciones iniciales del sistema (para simular)
x0=[0;0];
x=x0;
y=C*x0;
% Conjetura de condiciones iniciales para el filtro de kalman
xh=[0.5;-0.5]; % xh(0)
xp=xh; % xp(0)
S=eye(2,2); % S0

xr=[0;0];
yr=0;
% Simulacion
for k=1:length(t)-1
% sistema
u(k)=ref(k)*Nd-Kd*xh(:,k);         %CONTROL compensado
u(k)=ref(k)*Nd-Kd*xr(:,k);         %CONTROL no compensado
x(:,k+1)=A*x(:,k)+B*u(k)+G*sqrt(V)*randn;
y(k+1)=C*x(:,k+1)+sqrt(W)*randn;
% filtro de Kalman inestacionario
xp(:,k+1)=A*xh(:,k)+B*u(k); % estima a priori
L=(A*S*A'+G*V*G')*(C')*inv(C*(A*S*A'+G*V*G')*C'+W);
Lk(:,k)=L;
xh(:,k+1)=xp(:,k+1)+L*(y(k+1)-C*xp(:,k+1)); % estima
S=(eye(2)-L*C)'*(A*S*A'+G*V*G')*(eye(2)-L*C)'+L*W*L';


xr(:,k+1)= A*xr(:,k)+B*u(k)+Gd*(yr(k)-C*xr(:,k))+G*sqrt(V)*randn;
yr(k+1) = C*xr(:,k+1)+sqrt(W)*randn;

Ycomp(k)=C*((A-L*C*A)*(xh(:,k+1))+(B-L*C*B)*u(k)+L*y(k+1));
end

figure
plot(yr,'red')
hold on
plot(Ycomp,'green')
legend('Sin filtro','Con filtro')
grid on
