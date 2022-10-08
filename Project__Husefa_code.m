% Homework3 problem 2
close all
% vdes=25;
% N=300;
dt_init=.021;
N=500;
u_init=zeros(N,1);
% u_init(1:40)=2.5;
% for i=1:360
% u_init(i+40)=2.5-(i)/144;
% end
% plot(1:500,u_init);

options=optimoptions('fmincon','Display','iter','Algorithm','sqp','maxiterations',1000,'MaxFunctionEvaluations',1000000);
u_opt=fmincon(@objff,[u_init;dt_init],[],[],[],[],[],[],@nonlinconst,options);

%%

% dt=u_opt(end);
dt=u_opt(end);
u=u_opt(1:N);
x0=[11;0];
xopt=zeros(2,N);

Jt=21.17*N*dt;
Ju=dt*sumsqr(u);
J=Jt+Ju;

for i=1:N
    if i==1
    xopt(1,1)=x0(1)+dt*(u_opt(1)/1000-0.0981-1.2*x0(1)*x0(1)/1000);
    xopt(2,1)=x0(2)+dt*x0(1);
    else
    xopt(1,i)=xopt(1,i-1)+dt*(u_opt(i)/1000-0.0981-0.0012*xopt(1,i-1)*xopt(1,i-1));
    xopt(2,i)=xopt(2,i-1)+dt*xopt(1,i-1);
    end
end



%% Plotting the results
time=dt:dt:N*dt;
stairs(time,u_opt(1:N))
title('control sequence')
figure
plot(time,xopt(1,1:N),'r')
title('Velocity of car')
figure
plot(time,xopt(2,1:N),'b');
title('Distance travelled by car')

%%



function J=objff(dec_vars)
dt=dec_vars(end);
% dt=0.2;
N=500;
u=dec_vars(1:N);
x_init=[11;0];
x=zeros(2,N);
for i=1:N
    if i==1
    x(1,1)=x_init(1)+dt*(u(i)/1000-0.0981-1.2*x_init(1)*x_init(1)/1000);
    x(2,1)=x_init(2)+x_init(1)*dt;
    else
    x(1,i)=x(1,i-1)+dt*(u(i)/1000-0.0981-0.0012*x(1,i-1)*x(1,i-1));
    x(2,i)=x(2,i-1)+dt*x(1,i-1);
    end
end
Jt=14.33*N*dt;
Ju=dt*sumsqr(u);
J=Jt+Ju;


end

%non lin constraints
function [g,h]=nonlinconst(dec_vars)
dt=dec_vars(end);
% dt=0.2;
N=500;
u=dec_vars(1:N);
x0=[11;0];

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
g=zeros(3*N+1);
g=-dt;
 for i=1:N
      g(i+1)=x(1,i)-22.22;
      g(1+i+2*N)=-u(i);
       g(1+N+i)=u(i)-2.5;
 end
 
h=x(2,N)-200;
end

function theta=thetacalc(x)
if x>400 && x<=800
    theta=5*pi/180;
elseif x>800 && x<=1200
    theta=-5*pi/180;
else
    theta=0;
end
end