%Limpieza de pantalla
clear all
close all
clc

%Calculamos las matrices de transformación homogénea
H0=SE3;                             %Matriz identidad
H1=SE3(rotz(deg2rad(180)), [3 0 0]);
H2=SE3(roty(deg2rad(90))*rotx(deg2rad(150)), [0 0 2]);
%H3=SE3(, [-2 0 0]);

H0_1= H0*H1;
H1_2= H0_1*H2; %Matriz de transformación homogenea global de 2 a 0 

%Coordenadas de la estructura de translación y rotación
x=[0 3 3 0 0 0     0     0 0     3];
y=[0 0 0 0 0 5.196 5.196 0 5.196 0];
z=[0 0 2 2 0 0     2     2 2     2];

plot3(x, y, z,'LineWidth', 1.5); grid on;
axis([-1 4 -1 6 -1 2])
hold on;

%Graficamos la trama absoluta o global 
trplot(H0,'rgb','axis', [-1 1 -1 1 -1 1])
% 
% %Realizamos una animación para la siguiente trama
pause;
tranimate(H0, H0_1,'rgb','axis', [-1 1 -1 1 -1 1])
pause;
tranimate(H0_1, H1_2,'rgb','axis', [-1 1 -1 1 -1 1])
disp(H1_2)