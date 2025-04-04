clear all
close all
clc

%%% VELOCIDADES DE REFERENCIA %%%
% Trayectoria 1
%u = [ ones(1,20),         zeros(1,10), ones(1,20),          zeros(1,10),  ones(1,20),         zeros(1,10),  ones(1,20),         zeros(1,10)];
%w = [zeros(1,20), (pi/2) * ones(1,10), zeros(1,20), (pi/2) * ones(1,10), zeros(1,20), (pi/2) * ones(1,10), zeros(1,20), (pi/2) * ones(1,10)];

% Trayectoria 2
u = [zeros(1,10), ones(1,20),    zeros(1,10),             ones(1,20), zeros(1,10), ones(1,25), zeros(1,10), ones(1,30)];
w = [deg2rad(30)*ones(1,10), zeros(1,20)    deg2rad(130)*ones(1,10), zeros(1,20), -deg2rad(150)*ones(1,10), zeros(1,25), -deg2rad(70)*ones(1,10), zeros(1,30)];

%%% TIEMPO %%%
N = length(u);  % Muestras
tf = N/10;        % Tiempo de simulación en segundos (s)
ts = 0.1;       % Tiempo de muestreo en segundos (s)

t = linspace(0,tf,N);% t = 0:ts:tf;    % Vector de tiempo


%%% CONDICIONES INICIALES %%%
x1 = zeros(1, N+1); % Posición en el centro del eje que une las ruedas (eje x) en metros
y1 = zeros(1, N+1); % Posición en el centro del eje que una las ruedas (eje y) en metros
phi = zeros(1, N+1); % Orientación del robot en radianes (rad)

x1(1) = 0;  % Posición inicial eje x
y1(1) = 0;  % Posición inicial eje y
phi(1) = 0;  % Orientación inicial del robot

%%% PUNTO DE CONTROL %%%
hx = zeros(1, N+1); % Posición en el punto de control (eje x) en metros (m)
hy = zeros(1, N+1); % Posición en el punto de control (eje y) en metros (m)

hx(1) = x1(1);  % Posición en el punto de control del robot en el eje x
hy(1) = y1(1);  % Posición en el punto de control del robot en el eje y

pose_xp = zeros(1,N);
pose_yp = zeros(1,N);
pose_thp = w;

l = 0.09;
r = 0.05;

w_r = (2*u + w*l)/(2*r);
w_l = (2*u - w*l)/(2*r);

%%% BUCLE DE SIMULACION %%%
for k = 1:N
    % De la 34 a la 41 serán utiles con MCR2
    phi(k+1) = phi(k) + w(k)*ts;    % Integral numérica (método de Euler)

    %%% MODELO CINEMATICO %%%
    xp1 = u(k)*cos(phi(k+1));
    yp1 = u(k)*sin(phi(k+1));

    pose_xp(k) = xp1;
    pose_yp(k) = yp1;

    x1(k+1) = x1(k) + xp1*ts; % Integral numérica (método de Euler)
    y1(k+1) = y1(k) + yp1*ts; % Integral numérica (método de Euler)

    % Posición del robot con reespecto a l punto de control
    hx(k+1) = x1(k+1);
    hy(k+1) = y1(k+1);
end



% a) Configuracion de escena

scene=figure;  % Crear figura (Escena)
set(scene,'Color','white'); % Color del fondo de la escena
set(gca,'FontWeight','bold') ;% Negrilla en los ejes y etiquetas
sizeScreen=get(0,'ScreenSize'); % Retorna el tamaño de la pantalla del computador
set(scene,'position',sizeScreen); % Congigurar tamaño de la figura
camlight('headlight'); % Luz para la escena
axis equal; % Establece la relación de aspecto para que las unidades de datos sean las mismas en todas las direcciones.
grid on; % Mostrar líneas de cuadrícula en los ejes
box on; % Mostrar contorno de ejes
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)'); % Etiqueta de los eje

view([15 15]); % Orientacion de la figura
axis([-3 11 -3 10 0 2]); % Ingresar limites minimos y maximos en los ejes x y z [minX maxX minY maxY minZ maxZ]

% b) Graficar robots en la posicion inicial
scale = 4;
MobileRobot_5;
H1=MobilePlot_4(x1(1),y1(1),phi(1),scale);hold on;

% c) Graficar Trayectorias
H2=plot3(hx(1),hy(1),0,'r','lineWidth',2);

% d) Bucle de simulacion de movimiento del robot

step=1; % pasos para simulacion

for k=1:step:N

    delete(H1);    
    delete(H2);
    
    H1=MobilePlot_4(x1(k),y1(k),phi(k),scale);
    H2=plot3(hx(1:k),hy(1:k),zeros(1,k),'r','lineWidth',2);
    
    pause(ts);

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Graficas %%%%%%%%%%%%%%%%%%%%%%%%%%%%
graph=figure;  % Crear figura (Escena)
set(graph,'position',sizeScreen); % Congigurar tamaño de la figura
subplot(211)
plot(t,u,'b','LineWidth',2),grid('on'),xlabel('Tiempo [s]'),ylabel('m/s'),legend('u');
subplot(212)
plot(t,w,'r','LineWidth',2),grid('on'),xlabel('Tiempo [s]'),ylabel('[rad/s]'),legend('w');

figure;
subplot(3,1,1)
plot(t,pose_xp, 'g', 'LineWidth', 2), grid('on'),xlabel('Tiempo [s]'),ylabel('m/s'),legend('pose:xp');

subplot(3,1,2)
plot(t,pose_yp, 'b', 'LineWidth', 2), grid('on'),xlabel('Tiempo [s]'),ylabel('m/s'),legend('pose:yp');

subplot(3,1,3)
plot(t,pose_thp, 'r', 'LineWidth', 2), grid('on'),xlabel('Tiempo [s]'),ylabel('rad/s'),legend('pose:thp');

figure;
subplot(2,1,1)
plot(t,w_r, 'm', 'LineWidth', 2), grid('on'),xlabel('Tiempo [s]'),ylabel('rad/s'),legend('W_R');
subplot(2,1,2)
plot(t,w_l, 'c', 'LineWidth', 2), grid('on'),xlabel('Tiempo [s]'),ylabel('rad/s'),legend('W_L');