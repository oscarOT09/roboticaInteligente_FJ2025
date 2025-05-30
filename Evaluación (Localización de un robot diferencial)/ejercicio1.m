clear
close all
clc

sim_flag = false;

%%% VELOCIDADES DE REFERENCIA %%%
u = [ones(1,10), zeros(1,10),ones(1,10), zeros(1,10),ones(1,10), zeros(1,10),ones(1,10), zeros(1,10),ones(1,10), zeros(1,10),ones(1,10), zeros(1,10)];

w = [zeros(1,10), (pi/3)*ones(1,10),zeros(1,10), (pi/3)*ones(1,10),zeros(1,10), (pi/3)*ones(1,10),zeros(1,10), (pi/3)*ones(1,10),zeros(1,10), (pi/3)*ones(1,10),zeros(1,10), (pi/3)*ones(1,10)];

%%% TIEMPO %%%
ts = 0.1;
N = 120;
t = 0:ts:(N-1)*ts;  % esto garantiza que t(i) = i*ts exactamente

%%% CONDICIONES INICIALES %%%
x1 = zeros(1, N+1); % Posición en el centro del eje que une las ruedas (eje x) en metros
y1 = zeros(1, N+1); % Posición en el centro del eje que una las ruedas (eje y) en metros
phi = zeros(1, N+1); % Orientación del robot en radianes (rad)

x1(1) = -1;  % Posición inicial eje x
y1(1) = -5;  % Posición inicial eje y
phi(1) = 0;  % Orientación inicial del robot

%%% PUNTO DE CONTROL %%%
hx = zeros(1, N+1); % Posición en el punto de control (eje x) en metros (m)
hy = zeros(1, N+1); % Posición en el punto de control (eje y) en metros (m)

hx(1) = x1(1);  % Posición en el punto de control del robot en el eje x
hy(1) = y1(1);  % Posición en el punto de control del robot en el eje y

pose_xp = zeros(1,N);
pose_yp = zeros(1,N);
pose_thp = zeros(1,N);

%%% BUCLE DE CALCULOS %%%
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
pose_xp = round(pose_xp, 10);
pose_thp = rad2deg(pose_thp);

%%% SIMULACION %%%
if sim_flag
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
    axis([-3 3 -6 0 0 2]); % Ingresar limites minimos y maximos en los ejes x y z [minX maxX minY maxY minZ maxZ]
    
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
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Graficas %%%%%%%%%%%%%%%%%%%%%%%%%%%%
graph=figure;  % Crear figura (Escena)
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

% Índices originales que deseas extraer
indices_deseados = [1,11,21,31,41,51,61,71,81,91,101,111,120];

% Crear nuevo índice del 0 al 12
idx = (0:length(indices_deseados)-1)';

% Crear tabla filtrada con los valores deseados
tabla_pose = table(idx, pose_xp(indices_deseados)', pose_yp(indices_deseados)', pose_thp(indices_deseados)', ...
    'VariableNames', {'Indice', 'x_pose', 'y_pose', 'th_pose'});

% Mostrar tabla filtrada
disp(tabla_pose);