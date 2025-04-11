% Actividad 4.1 (Evaluación)
% Oscar Ortiz Torres A01769292

clear
close all
clc

%%% VELOCIDADES DE REFERENCIA %%%
op_tray = 2;

if op_tray == 1
    ts = 0.01;
    x_ref = 0:ts:5;
    y_ref = 2 * sin(x_ref.^2);
elseif op_tray == 2
    ts = 0.01;
    theta = 0:ts:(2*pi);
    x_ref = 4 * cos(theta);
    y_ref = 4 * sin(theta);
elseif op_tray == 3
    ts = 0.1;
    x_ref = -6:ts:6;
    y_ref = zeros(size(x_ref));
    
    for i = 1:length(x_ref)
        x = x_ref(i);
        if x <= 0
            y_ref(i) = x;
        elseif x <= 1
            y_ref(i) = 3*x;
        elseif x < 4
            y_ref(i) = 3;
        else
            y_ref(i) = 2*x - 5;
        end
    end
end

dx = gradient(x_ref, ts);
dy = gradient(y_ref, ts);
u = sqrt(dx.^2 + dy.^2);
theta = atan2(dy, dx);
w = gradient(theta, ts);

%%% TIEMPO %%%
N = length(u);  % Muestras
tf = N/10;        % Tiempo de simulación en segundos (s)

t = linspace(0,tf,N); % t = 0:ts:tf;    % Vector de tiempo

x1 = zeros(1, N+1); % Posición en el centro del eje que une las ruedas (eje x) en metros
y1 = zeros(1, N+1); % Posición en el centro del eje que una las ruedas (eje y) en metros
phi = zeros(1, N+1); % Orientación del robot en radianes (rad)
%%% CONDICIONES INICIALES %%%

if op_tray ~= 3
    x1(1) = 0;  % Posición inicial eje x
    y1(1) = 0;  % Posición inicial eje y
    phi(1) = 0;  % Orientación inicial del robot
else
    x1(1) = x_ref(1);
    y1(1) = y_ref(1);
    phi(1) = theta(1);
end 


%%% PUNTO DE CONTROL %%%
hx = zeros(1, N+1); % Posición en el punto de control (eje x) en metros (m)
hy = zeros(1, N+1); % Posición en el punto de control (eje y) en metros (m)

hx(1) = x1(1);  % Posición en el punto de control del robot en el eje x
hy(1) = y1(1);  % Posición en el punto de control del robot en el eje y

pose_xp = zeros(1,N);
pose_yp = zeros(1,N);
pose_thp = zeros(1,N);

%%% BUCLE DE SIMULACION %%%
for k = 1:N
    
    phi(k+1) = phi(k) + w(k)*ts;    % Integral numérica (método de Euler)

    %%% MODELO CINEMATICO %%%
    xp1 = u(k)*cos(phi(k+1));
    yp1 = u(k)*sin(phi(k+1));

    x1(k+1) = x1(k) + xp1*ts; % Integral numérica (método de Euler)
    y1(k+1) = y1(k) + yp1*ts; % Integral numérica (método de Euler)
    
    % Pose
    pose_xp(k) = x1(k);
    pose_yp(k) = y1(k);
    pose_thp(k) = phi(k);

    % Posición del robot con reespecto a l punto de control
    hx(k+1) = x1(k+1);
    hy(k+1) = y1(k+1);
end

pose_thp = rad2deg(pose_thp);

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
axis([-10 10 -10 10 0 5]); % Ingresar limites minimos y maximos en los ejes x y z [minX maxX minY maxY minZ maxZ]

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
plot(t,pose_xp, 'g', 'LineWidth', 2), grid('on'),xlabel('Tiempo [s]'),ylabel('m'),legend('pose:xp');

subplot(3,1,2)
plot(t,pose_yp, 'b', 'LineWidth', 2), grid('on'),xlabel('Tiempo [s]'),ylabel('m'),legend('pose:yp');

subplot(3,1,3)
plot(t,pose_thp, 'r', 'LineWidth', 2), grid('on'),xlabel('Tiempo [s]'),ylabel('Grados (°)'),legend('pose:thp');