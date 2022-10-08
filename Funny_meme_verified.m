% x = [0 :100]
% y =[0:100]
close all
clc
clear
x = linspace(-15,15,100);   %%% Setting the range to [-1 1]
y = linspace(-15,15,100);

[x,y]=meshgrid(x,y);
N1 = ((x+4).^2 + (y+4).^2 ).^2;
N2 = ((x-4).^2 + (y-4).^2 ).^2;

z = exp(-N1./1000)+exp(-N2./1000) +(0.1).*(exp(-N1./1)+exp(-N2./1));
% [~,index]= max(z)
% plot3(x,y,z)
mesh(x,y,z)
%s = surf(x,y,z)
%s.EdgeColor='none'
zlim([0 3.5])