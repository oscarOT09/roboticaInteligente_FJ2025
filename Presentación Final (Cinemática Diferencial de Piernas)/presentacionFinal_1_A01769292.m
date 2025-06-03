%Limpieza de pantalla
clear all
close all
clc

%Calculamos las matrices de transformación homogénea
H0=SE3;                             %Matriz identidad
H1=SE3(rotx(deg2rad(90)), [0 0 5]);
H2=SE3(rotz(0), [5 0 0]);
H3=SE3(rotz(0), [5 0 0]);
H4=SE3(rotz(0), [5 0 0]);

H0_1= H0*H1;
H1_2= H0_1*H2;
H2_3= H1_2*H3;
H3_4= H2_3*H4; % Matriz de transformación homogenea global de 4 a 0 

%Coordenadas de la estructura de translación y rotación
x=[0 0 5 10 15];
y=[0 0 0  0  0];
z=[0 5 5  5  5];

plot3(x, y, z,'LineWidth', 1.5); grid on;
axis([-1 16 -1 1 0 7]);
hold on;
view(45, 30)

%Graficamos la trama absoluta o global 
trplot(H0,'rgb','axis', [-1 1 -1 1 -1 1])
% 
% %Realizamos una animación para la siguiente trama
pause;
tranimate(H0, H0_1,'rgb','axis', [-1 1 -1 1 -1 1])
pause;
tranimate(H0_1, H1_2,'rgb','axis', [-1 1 -1 1 -1 1])
pause;           
tranimate(H1_2, H2_3,'rgb','axis', [-1 1 -1 1 -1 1])
pause;
tranimate(H2_3, H3_4,'rgb','axis', [-1 1 -1 1 -1 1])
disp(H3_4)
