clear all
close all
clc

%%% VELOCIDADES DE REFERENCIA %%%

x_vec = [4, 6, 8, 6, 4, 2, 0, 2, 4, 4, 2, 2, 0, 2, 0, 2, 2, 4, 6, 6, 8, 6, 8, 6, 6, 6, 4];
y_vec = [1, 1, 3, 3, 1, 3, 3, 1, 1, 5, 3, 5, 5, 7, 9, 9, 11, 9, 11, 9, 9, 7, 5, 5, 5, 3, 5];

%%% AUTOMATIZACION DE u y w PARA MOVIMIENTO EN LINEAS RECTAS %%%
ts = 0.1;          % Tiempo de muestreo
v_lineal = 1.0;    % Velocidad lineal
w_angular = pi/4;  % Velocidad angular (45°/s)

u = [];
w = [];

theta_prev = 0;

for i = 1:length(x_vec)-1
    dx = x_vec(i+1) - x_vec(i);
    dy = y_vec(i+1) - y_vec(i);

    % Ángulo hacia el siguiente punto
    theta = atan2(dy, dx);

    % Calcular ángulo a girar
    delta_theta = wrapToPi(theta - theta_prev);
    
    % ----------------------
    % Paso 1: Giro
    % ----------------------
    t_giro = abs(delta_theta) / w_angular;
    pasos_giro = round(t_giro / ts);
    
    u = [u, zeros(1, pasos_giro)];
    w = [w, sign(delta_theta) * w_angular * ones(1, pasos_giro)];

    % ----------------------
    % Paso 2: Movimiento recto
    % ----------------------
    dist = sqrt(dx^2 + dy^2);
    t_linea = dist / v_lineal;
    pasos_linea = round(t_linea / ts);

    % VELOCIDADES
    u = [u, v_lineal * ones(1, pasos_linea)];
    w = [w, zeros(1, pasos_linea)];

    % Actualiza orientación
    theta_prev = theta;
end


%%% TIEMPO %%%
N = length(u);      % Muestras
tf = N*ts;          % Tiempo de simulación en segundos
t = linspace(0, tf, N);

%%% CONDICIONES INICIALES %%%
x1 = zeros(1, N+1);
y1 = zeros(1, N+1);
phi = zeros(1, N+1);

x1(1) = -9;
y1(1) = 10;
phi(1) = 0;

%%% PUNTO DE CONTROL %%%
hx = zeros(1, N+1);
hy = zeros(1, N+1);

hx(1) = x1(1);
hy(1) = y1(1);

l = 0.18;
r = 0.05;

w_l = (2*u - w*l)/(2*r);
w_r = (2*u + w*l)/(2*r);

pose_xp = zeros(1,N);
pose_yp = zeros(1,N);
pose_thp = zeros(1,N);

%%% BUCLE DE SIMULACION %%%
for k = 1:N
    
    phi(k+1) = phi(k) + w(k)*ts;

    xp1 = u(k)*cos(phi(k+1));
    yp1 = u(k)*sin(phi(k+1));

    x1(k+1) = x1(k) + xp1*ts;
    y1(k+1) = y1(k) + yp1*ts;
    
    % POSE
    pose_xp(k) = x1(k);
    pose_yp(k) = y1(k);
    pose_thp(k) = phi(k);

    hx(k+1) = x1(k+1);
    hy(k+1) = y1(k+1);
end

pose_thp = rad2deg(pose_thp);

% a) Configuracion de escena

scene = figure;
set(scene,'Color','white');
set(gca,'FontWeight','bold');
sizeScreen = get(0,'ScreenSize');
set(scene,'position',sizeScreen);
camlight('headlight');
axis equal;
grid on;
box on;
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
view([15 15]);

margen = 2;
ymin = min(y1) - margen;
ymax = max(y1) + margen;
xmin = min(x1) - margen;
xmax = max(x1) + margen;

axis([xmin xmax ymin ymax 0 2]);

% b) Graficar robots en la posicion inicial
scale = 4;
MobileRobot_5;
H1 = MobilePlot_4(x1(1),y1(1),phi(1),scale); hold on;

plot(x_vec, y_vec, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'b');

% c) Graficar Trayectorias
H2 = plot3(hx(1),hy(1),0,'r','lineWidth',2);

% d) Bucle de simulacion de movimiento del robot

step = 1;
for k = 1:step:N
    delete(H1);    
    delete(H2);
    
    H1 = MobilePlot_4(x1(k),y1(k),phi(k),scale);
    H2 = plot3(hx(1:k),hy(1:k),zeros(1,k),'r','lineWidth',2);
    
    pause(ts);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Graficas %%%%%%%%%%%%%%%%%%%%%%%%%%%%

graph = figure;
set(graph,'position',sizeScreen);

subplot(211)
plot(t,u,'b','LineWidth',2),grid('on'),xlabel('Tiempo [s]'),ylabel('m/s'),legend('u');
subplot(212)
plot(t,w,'r','LineWidth',2),grid('on'),xlabel('Tiempo [s]'),ylabel('[rad/s]'),legend('w');

figure;
subplot(3,1,1)
plot(t,pose_xp, 'g', 'LineWidth', 2), grid('on'),xlabel('Tiempo [s]'),ylabel('m'),legend('pose:xp');

subplot(3,1,2)
plot(t,pose_yp, 'b', 'LineWidth', 2), grid('on'),xlabel('Tiempo [s]'),ylabel('m'),legend('pose:yp');

subplot(3,1,3)
plot(t,pose_thp, 'r', 'LineWidth', 2), grid('on'),xlabel('Tiempo [s]'),ylabel('Grados (°)'),legend('pose:thp');
