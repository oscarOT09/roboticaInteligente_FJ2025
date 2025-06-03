%% Presentación Final | Cinemática diferencial de piernas
% Oscar Ortiz Torres A01769292

%Limpieza de pantalla
clear
close all
clc

% Matrices de transformación homogénea
H0=SE3(rotx(deg2rad(180)), [15 10 15]);
H1=SE3(rotz(0), [0 0 0]);
H2=SE3(rotx(deg2rad(-90))*roty(deg2rad(90)), [0 0 0]);
H3=SE3(rotx(deg2rad(90))*roty(deg2rad(-90)),[0 0 0]);
H4=SE3(rotz(0), [7.5 -3 0]);
H5=SE3(rotz(deg2rad(90)), [7.5 -4 0]);
H6=SE3(rotx(deg2rad(90))*roty(deg2rad(90)),[0,0,0]);
H7=SE3(rotz(0), [0 0 5]);

H0_1= H0*H1;
H1_2= H0_1*H2;
H2_3= H1_2*H3;
H3_4= H2_3*H4;
H4_5= H3_4*H5;
H5_6= H4_5*H6;
H6_7= H5_6*H7; % Matriz de transformación homogenea global de 7 a 0 

%Coordenadas de la estructura de translación y rotación
x=[15  12  8 13];
y=[10  10 10 10];
z=[15 7.5  0  0];

plot3(x, y, z,'LineWidth', 1.5); grid on;
axis([5 17.5 7.5 12.5 -5 20]);
hold on;
view(45, 30)


% Graficación de la trama global
trplot(H0, 'rgb', 'axis', [-1 1 -1 1 -1 1])

% Animación para cada trama
pause;
tranimate(H0, H0_1, 'rgb', 'axis', [-1 1 -1 1 -1 1])
pause;
tranimate(H0_1, H1_2, 'rgb', 'axis', [-1 2 -1 2 -1 2])
pause;
tranimate(H1_2, H2_3, 'rgb', 'axis', [-1 3 -1 3 -1 3])
pause;
tranimate(H2_3, H3_4, 'rgb', 'axis', [-1 1 -1 1 -1 1])
pause;
tranimate(H3_4, H4_5, 'rgb', 'axis', [-1 1 -1 1 -1 1])
pause;
tranimate(H4_5, H5_6, 'rgb', 'axis', [-1 2 -1 2 -1 2])
pause;
tranimate(H5_6, H6_7, 'rgb', 'axis', [-1 1 -1 1 -1 1])
disp(H6_7)
