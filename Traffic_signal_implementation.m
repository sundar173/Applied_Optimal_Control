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
u0=10*ones(N,1);
%%%%%%%%%%%%%%%%%%%%%%  FMINCON  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
options = optimoptions('fmincon','Display','iter','Algorithm','sqp','maxiterations',1000,'MaxFunctionEvaluations',1000000);
[u_opt] = fmincon(@obj_func,[u0;T0],[],[],[],[],[],[],@const,options);
T=u_opt(end);
v0=1; %%%%%%%%%% Initial Speed %%%%%%%%%%%%%
x0=0;
% v(1)=v0+(1/m)*(u_opt(1)-(Crr*m*g)-(0.5*rho*(v0^2)*Cd*Aref))*T;
v(1)=v0+u_opt(1)*T;
x(1)=x0+v0*T;
    for i=2:N
        v(i)=v(i-1)+u_opt(i)*T;
%         v(i)=v(i-1)+(1/m)*(u_orpt(i)-(Crr*m*g)-(0.5*rho*v(i-1)^2*Cd*Aref))*T;
        x(i)=x(i-1)+v(i-1)*T;
    end
%% Generating plots  
t=T:T:N*T;
t0=T;
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
f=0;  %%%%%%%% f=1 if the starting signal is green and f=0 if the starting signal is red
%Traffic signal
if f==1
    k=60;
    D=50;
    tg(1)=t0;
    tr(1)=t0+D;
    for i=2:10
        tg(i)=tg(i-1)+k;
        tr(i)=tr(i-1)+k;
    end
    xlim([0 70])
    yticks(0:25:200)
    for i=1:3
        taxis_G(i,:)=[tg(i):1:tr(i)];
         taxis_R(i,:)=[tr(i):1:tg(i+1)];
    end
    y1=200*ones(size(taxis_G));
    y2=200*ones(size(taxis_R));
    n=1;   %%%%%% no of traffic cycles to plot
    for i=1:n
           plot(taxis_G(i,:),y1,'g','LineWidth',2)
           hold on 
         plot(taxis_R(i,:),y2,'r','LineWidth',2)
         hold on
    end
    hold on
    plot(t,x)
    ylabel('Travelled distance ,m')
    xlabel('Time(T)')
    title('Distance vs Time')
end
if f==0
    k=60;
    D=20;
    tr(1)=t0;
    tg(1)=t0+D;
    for i=2:10
        tr(i)=tr(i-1)+k;
        tg(i)=tg(i-1)+k;
    %     timevector_g(:,:,i-1)=[tg(i-1):tr(i-1)];
    %     timevector_r(:,:,i-1)=[tr(i-1):tg(i)];
    end
    % y=150*ones(size(timevector_g));
    % plot(timevector_g(1,:,1:3),y,'--g')
    xlim([0 70])
    yticks(0:25:200)
    for i=1:3
        taxis_R(i,:)=tr(i):1:tg(i);
         taxis_G(i,:)=tg(i):1:tr(i+1);
    end
    y1=200*ones(size(taxis_R));
    y2=200*ones(size(taxis_G));
    n=1;   %%%%%% no of traffic cycles to plot
    for i=1:n
           plot(taxis_R(i,:),y1,'r','LineWidth',2)
           hold on 
         plot(taxis_G(i,:),y2,'g','LineWidth',2)
         hold on
    end
    hold on
    plot(t,x)
    ylabel('Travelled distance ,m')
    xlabel('Time(T)')
    title('Distance vs Time')
end
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
v0=21;  %----------------------------------------Initial velocity
f=0;
v(1)=v0+u(1)*T;
% v(1)=v0+(1/m)*(u(1)-(Crr*m*g)-(0.5*rho*(v0^2)*Cd*Aref))*T;
    for i=2:N
        v(i)=v(i-1)+u(i)*T;
%         v(i)=v(i-1)+(1/m)*(u(i)-(Crr*m*g)-(0.5*rho*v(i-1)^2*Cd*Aref))*T;
    end
% J=sum(sum(u.*v))*T;
% J=(v*u)*T;

Rho_t=0.0133;
Rho_u=9.2798e-4;
Rho=Rho_t/Rho_u;
Jt=Rho*(N*T*signal_check(N,T,f));
Ju=sumsqr(u)*T;
J=Jt+Ju;
% Rho_t=0.55;
% Rho_u=0.45;
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
x0=[21 0]'; %------------------------------Initial state
N=500;
f=1;
T=dec_vars(end);
u=dec_vars(1:N);
vmin=2.78;
vmax=22.22;
umin=-2.9; %% Minimum value of acceleration
umax=2.5;  %% Maximum value of acceleration
% x(1,1)=x0(1)+(1/m)*(u(1)-Crr*m*g-0.5*rho*x0(1)^2*Cd*Aref)*T;
x(2,1)=x0(2)+x0(1)*T;
x(1,1)=x0(1)+u(1)*T;
% x(2,1)=x0(2)+x0(1)*T;
    for i=2:N
        x(1,i)=x(1,i-1)+u(i)*T;
        x(2,i)=x(2,i-1)+x(1,i-1)*T;
        
%         x(1,i)=x(1,i-1)+(1/m)*(u(i)-Crr*m*g-0.5*rho*x(1,i-1)^2*Cd*Aref)*T;
%         x(2,i)=x(2,i-1)+x(1,i-1)*T;
    end
for i=1:N
    g(i)=-x(1,i)+vmin;
    g(i+N)=x(1,i)-vmax;
    g(i+2*N)=u(i)-umax;
    g(i+3*N)=-u(i)+umin;
    g(4*N+1)=-T;
    h(1)=x(2,N)-2203;
%     if f==0 %% Starts at red
%         t0=T;
%         k=60;
%         D=20;
%         D_r=abs(k-D);
%         tr(1)=t0;
%         tg(1)=t0+D;
% %         Rho=Rho_t/Rho_u;
% %         Jt=Rho*(N*T);
% %         Ju=sumsqr(u)*T;
%          if N*T<D
%              dt=D-N*T;
% %              J=Jt+Ju+dt;
%              h(2)=dt;
%          end
        %%%%%%%%%%% Adding a terminal cost if crashed into a red signal
    %     if N*T>D
    %         dt=D_r-(N*T-D);
    %         J=Jt+Ju+dt;
    %     end 
%       end
end
end

function term_cost=signal_check(N,T,f)
k=60;
t_arr=T*N;
    if (f==1)
        D=50;
        if (t_arr>=D && t_arr<=k) 
            term_cost=k-t_arr;
        elseif (t_arr>=D+k && t_arr<=2*k)
            term_cost=2*k-t_arr;
        else
            term_cost=0;
        end
    
    elseif(f==0)
        D=20;
        if t_arr<D
            term_cost=D-t_arr;
        end
    end

end