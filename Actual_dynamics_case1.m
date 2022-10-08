close all
clc
clear
clear;
dt_init=.02;
L=200;
N=1000;
u_init=0*ones(N,1);
f=1;
% u_init(1:40)=2.5;
% for i=1:500
% u_init(i,1)=-0.08*(1-i/500);
% end
% plot(1:500,u_init);

options=optimoptions('fmincon','Display','iter','Algorithm','sqp','maxiterations',1000,'MaxFunctionEvaluations',100000,'OptimalityTolerance',0.0000001);
u_opt=fmincon(@objff,[u_init;dt_init],[],[],[],[],[],[],@nonlinconst,options);

%%

% dt=u_opt(end);
dt=u_opt(end);
u=u_opt(1:N);
x0=[18.6182;0];
xopt=zeros(2,N);

rho=0.9549;
rhot=rho*2.78/L;
rhou=(1-rho)/(22.22-2.78)/2.5;
rhou=rhou/1000000;
Jt=rhot*(N*dt+signalcheck(N,dt));
 Ju=rhou*dt*sumsqr(u);

J=Jt+Ju;

for i=1:N
    if i==1
    xopt(1,1)=x0(1)+dt*(u_opt(1)/1000-0.0981-0.0012*x0(1)*x0(1));
    xopt(2,1)=x0(2)+dt*x0(1);
    else
    xopt(1,i)=xopt(1,i-1)+dt*(u_opt(i)/1000-0.0981-0.0012*xopt(1,i-1)*xopt(1,i-1));
    xopt(2,i)=xopt(2,i-1)+dt*xopt(1,i-1);
    end
end



%% Plotting the results
time=dt:dt:N*dt;
subplot(2,1,1)
% figure
stairs(time,u_opt(1:N))
title('control sequence')
 ylabel('control sequence- acceleration')
 xlabel('Time(T)')
% figure
subplot(2,1,2)
plot(time,xopt(1,1:N))
title('Velocity of car')
 ylabel('Velocity')
 xlabel('Time(T)')
 figure
% plot(time,xopt(2,1:N),'g');
% title('Distance travelled by car')
% hold on

%% Traffic signal
t0=dt;
t=time;
x=xopt;
if f==1
    k=60;
    D=30;
    tg(1)=t0;
    tr(1)=t0+D;
    for i=2:10
        tg(i)=tg(i-1)+k;
        tr(i)=tr(i-1)+k;
    end
%     xlim([0 70])
    ylim([0 L])
%     yticks(0:25:L)
    for i=1:3
        taxis_G(i,:)=[tg(i):1:tr(i)];
         taxis_R(i,:)=[tr(i):1:tg(i+1)];
    end
    y1=L*ones(size(taxis_G));
    y2=0*ones(size(taxis_R));
    n=2;   %%%%%% no of traffic cycles to plot
    for i=1:n
           plot(taxis_G(i,:),y1,'g','LineWidth',2)
           hold on 
           vert1=taxis_G(i,end);
           xline(vert1,'--r');
            plot(taxis_R(i,:),y2,'r','LineWidth',2)
            vert2=taxis_R(i,end);
           xline(vert2,'--g');
         hold on
    end
    hold on
plot(time,xopt(2,1:N));
% xlim([0 70])
    ylim([0 L])
%     yticks(0:25:L)
% title('Distance travelled by car')
    ylabel('Travelled distance ,m')
    xlabel('Time(T)')
    title('Distance vs Time')
hold on
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
%     xlim([0 70])
    ylim([0 L])
    yticks(0:25:L)
    for i=1:3
        taxis_R(i,:)=tr(i):1:tg(i);
         taxis_G(i,:)=tg(i):1:tr(i+1);
    end
    y1=0*ones(size(taxis_R));
    y2=L*ones(size(taxis_G));
    n=1;   %%%%%% no of traffic cycles to plot
    for i=1:n
           plot(taxis_R(i,:),y1,'r','LineWidth',2)
           hold on 
           vert2=taxis_R(i,end);
           xline(vert2,'--g');
         plot(taxis_G(i,:),y2,'g','LineWidth',2)
         hold on
         vert1=taxis_G(i,end);
           xline(vert1,'--r');
    end
    hold on
    plot(time,xopt(2,1:N));
    ylim([0 L])
    yticks(0:25:L)
% title('Distance travelled by car')
    ylabel('Travelled distance ,m')
    xlabel('Time(T)')
    title('Distance vs Time')
hold on
%     plot(t,x)
%     ylabel('Travelled distance ,m')
%     xlabel('Time(T)')
%     title('Distance vs Time')
end

%%



function J=objff(dec_vars)
dt=dec_vars(end);
% dt=0.2;
N=500;
L=200;
u=dec_vars(1:N);
x_init=[18.6182;0];
x=zeros(2,N);
for i=1:N
    if i==1
    x(1,1)=x_init(1)+dt*(u(i)/1000-0.0981-0.0012*x_init(1)*x_init(1));
    x(2,1)=x_init(2)+x_init(1)*dt;
    else
    x(1,i)=x(1,i-1)+dt*(u(i)/1000-0.0981-0.0012*x(1,i-1)*x(1,i-1));
    x(2,i)=x(2,i-1)+dt*x(1,i-1);
    end
end
rho=0.9549;
rhot=rho*2.78/L;
rhou=(1-rho)/(22.22-2.78)/2.5;
rhou=rhou/1000000;
% rhot=14.3322;
% rhou=1;
Jt=rhot*(N*dt+signalcheck(N,dt));
Ju=rhou*dt*sumsqr(u);
J=(Jt+Ju);


end

%non lin constraints
function [g,h]=nonlinconst(dec_vars)
dt=dec_vars(end);
% dt=0.2;
L=200;
N=length(dec_vars)-1;
u=dec_vars(1:end);
x0=[18.6182;0];

x=zeros(1,N);
for i=1:N
    if i==1
    x(1,i)=x0(1)+dt*(u(1)/1000-0.0981-0.0012*x0(1)*x0(1));
    x(2,i)=x0(2)+x0(1)*dt;
    else
    x(1,i)=x(1,i-1)+dt*(u(i)/1000-0.0981-0.0012*x(1,i-1)*x(1,i-1));
    x(2,i)=x(2,i-1)+x(1,i-1)*dt;
    end
end
% g(1)=1600-x(2,300);
% g(2)=25-x(1,300);
% g=zeros(1,600);
g=zeros(3*N+1,1);
g(1)=-dt;
 for i=1:N
      g(i+1)=x(1,i)-22.22;
      g(1+i+2*N)=-u(i)/2900-1;
       g(1+N+i)=u(i)/2500-1;
 end
% g(3*N+2)=-dt*N+120;
% g(3*N+3)=dt*N-90;
h=x(2,N)-L;
check=signaltrigger(N,dt);
if check==1
g(3*N+2)=x(1,N)-0.01;
g(3*N+3)=-x(1,N)-0.01;
else
g(3*N+2)=x(1,N)-22.22;
g(3*N+3)=-x(1,N);
end
end

function signal=signalcheck(N,dt)
time=dt*N;
if (time>50 && time<60)  
       signal=60-time;
%    signal=1000;
elseif (time>90 && time<120)
     signal=120-time;
%    signal=1000;
else
    signal=0;
end
end

function trigger=signaltrigger(N,dt)
time=dt*N;
if (time>50 && time<60) 
    trigger=1;
elseif (time>90 && time<120)
    trigger=1;
else
    trigger=0;
end
end
