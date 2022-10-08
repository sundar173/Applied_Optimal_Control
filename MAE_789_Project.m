                            %%%%%%%%%%%Project%%%%%%%%%%%%%%%%%%%%
                                      %-------%
clc
close all
clear all
% umax=3000;
% umin=0;
%%%%%%%%%%%%%%%%%%%%%%%%% System parameters%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
umin=0; %% Minimum value of acceleration
umax=2.5;  %% Maximum value of acceleration
m=1000;    %% Mass
Crr=0.01;  %% Rolling Resistance
Cd=0.4;    %% Coefficient of drag
g=9.8;     %% acceleration due to gravity 
rho=1.2;   %% Density 
Aref=5;    %% Reference area
T0=0.021;
N=500;
u0=10*ones(N,1)*m;
%%%%%%%%%%%%%%%%%%%%%%  FMINCON  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
options = optimoptions('fmincon','Display','iter','Algorithm','sqp','maxiterations',1000,'MaxFunctionEvaluations',1000000);
[u_opt] = fmincon(@obj_func,[u0;T0],[],[],[],[],[],[],@const,options)
T=u_opt(end);
v0=11; %%%%%%%%%% Initial Speed %%%%%%%%%%%%%
x0=0;
v(1)=v0+(1/m)*(u_opt(1)-(Crr*m*g)-(0.5*rho*(v0^2)*Cd*Aref))*T;
% v(1)=v0+u_opt(1)*T;
x(1)=x0+v0*T;
    for i=2:N
%         v(i)=v(i-1)+u_opt(i)*T;
        v(i)=v(i-1)+(1/m)*(u_opt(i)-(Crr*m*g)-(0.5*rho*v(i-1)^2*Cd*Aref))*T;
        x(i)=x(i-1)+v(i-1)*T;
    end
%% Generating plots  
t=T:T:N*T
stairs(t,u_opt(1:N))
ylabel('Optimum Control sequence u*')
xlabel('Time(T)')
title('Optimum Control sequence vs time')    
   
figure
plot(t,v)
ylabel('Vehicle velocity ,m/s')
xlabel('Time(T)')
title('Vehicle Velocity profile')
figure 
 plot(t,x)
 ylabel('Travelled distance ,m')
xlabel('Time(T)')
title('Distance vs Time')
%% Objective function
function J=obj_func(dec_vars)
m=1000;    %% Mass
Crr=0.01;  %% Rolling Resistance
Cd=0.4;    %% Coefficient of drag
g=9.8;     %% acceleration due to gravity 
rho=1.2;   %% Density 
Aref=5;    %% Reference area
N=500;
T=dec_vars(end);
u=dec_vars(1:N);
v=zeros(1,N);
v0=11;
% v(1)=v0+u(1)*T;
v(1)=v0+(1/m)*(u(1)-(Crr*m*g)-(0.5*rho*(v0^2)*Cd*Aref))*T;
    for i=2:N
%         v(i)=v(i-1)+u(i)*T;
        v(i)=v(i-1)+(1/m)*(u(i)-(Crr*m*g)-(0.5*rho*v(i-1)^2*Cd*Aref))*T;
    end
% J=sum(sum(u.*v))*T;
% J=(v*u)*T;
Rho_t=0.0133;
Rho_u=9.2798e-4;
% Rho_t=0.55;
% Rho_u=0.45;
Rho=Rho_t/Rho_u;
Jt=Rho*(N*T);
Ju=sumsqr(u)*T;
J=Jt+Ju;
return
end
%% Constraint set function
function [g,h]=const(dec_vars)
m=1000;
Crr=0.01;
Cd=0.4;
g=9.8;
rho=1.2;
Aref=5;
T=0.1;
x0=[11 0]';
N=500;
T=dec_vars(end);
u=dec_vars(1:N);
vmin=2.78;
vmax=22.22;
umin=-2.9*m; %% Minimum value of acceleration
umax=2.5*m;  %% Maximum value of acceleration
x(1,1)=x0(1)+(1/m)*(u(1)-Crr*m*g-0.5*rho*x0(1)^2*Cd*Aref)*T;
% x(2,1)=x0(2)+x0(1)*T;
% x(1,1)=x0(1)+u(1)*T;
x(2,1)=x0(2)+x0(1)*T;
    for i=2:N
%         x(1,i)=x(1,i-1)+u(i)*T;
%         x(2,i)=x(2,i-1)+x(1,i-1)*T;
        
        x(1,i)=x(1,i-1)+(1/m)*(u(i)-Crr*m*g-0.5*rho*x(1,i-1)^2*Cd*Aref)*T;
        x(2,i)=x(2,i-1)+x(1,i-1)*T;
    end
for i=1:N
    g(i)=-x(1,i)+vmin;
    g(i+N)=x(1,i)-vmax;
    g(i+2*N)=u(i)/-umax;
    g(i+3*N)=-(u(i)/umin);
end
g(4*N+1)=-T;
h=x(2,N)-200;
end
