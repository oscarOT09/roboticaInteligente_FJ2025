% Equipo ROSario

% Limpieza de pantalla
clear 
close all
clc

% Definir que figura se quiere

% Op = 1 -> Gato
% Op = 2 -> Murcielago
% Op = 3 -> Mariposa

op = 1;

% Vectores de componentes X y Y de las coordenadas objetivos
if op == 1
    x_vec = [1, 5, 5, 3, 3, 4, 6, 6, 9, 9, 8, 8, 9, 11, 13, 15, 16, 16, 14, 10,  8, 8, 6, 4, 2,  2,  3,  5,  5,  3,  0, 0, 1];
    y_vec = [0, 0, 2, 2, 3, 4, 4, 0, 0, 2, 2, 5, 4,  3,  3,  4,  5, 11,  9,  9, 11, 7, 9, 9, 7, 10, 10, 12, 13, 13, 10, 6, 0];
elseif op == 2
    x_vec = [1, 2, 3, 3, 4.8, 6, 8, 10, 12, 13, 13, 15, 19, 22, 24, 26, 24.8,22.8,22,18.8, 17,16,15, 15,13,11, 8.6, 8, 6.8, 5, 1, 0, -4, -6, -8, -10, -11, -12, -13, -14, -17, -18, -21.8, -24, -24.8, -26.2, -25, -23, -21, -19, -14.8, -12, -10, -7, -4, -1, -1, -2, 0, 1];
    y_vec = [0, 0, 1, 3.3, -1, -2, 0, 4, 8, 13, 16, 15, 15, 15, 15, 17, 14, 10, 6, 5, 3,1, -2, -7, -8, -10, -12, -13.8, -18, -15.8, -14, -14, -15, -17, -15, -13, -12, -12, -12, -13, -15, -15, -13, -12, -12, -13, -11, -8, -5, 0, -2, -4, -5, -6, -6, -6, -5, 1, -1, 0];
elseif op == 3
    x_vec = [-0.8, 0, 0.6, 2, 3.2, 4, 3.2, 2.6, 4, 4, 2.6, 2, 0.6, 0, -0.6, -2, -2.6, -4, -4, -2.6, -3.2, -4, -3.2, -2, -0.6, 0, 0.8];
    y_vec = [6.8, 6, 5.2, 5.6, 5.6, 5, 2.8, 2, 0.8, 0, -0.6, 0, 0.8, 0, 0.8, 0, -0.6, 0, 0.8, 2, 2.8, 4, 5.6, 5.6, 5.2, 6, 6.8];
end

%%%%%% COntron Lazo Abierto %%%%%%%%%%%%%

% Parámetros del movimiento
ts = 0.01;       % Tiempo de muestreo
v = 10;         % Velocidad lineal
w = 10;         % Velocidad angular

% Inicialización del robot
x = x_vec(1); y = y_vec(1); theta = 0; % Posición y orientación inicial
x1 = []; y1 = []; phi = []; % Trayectoria simulada
hx = []; hy = [];          % Historia del centro del robot

for i = 1:(length(x_vec)-1)
    x_goal = x_vec(i+1);
    y_goal = y_vec(i+1);
    
    % ROTACIÓN EN EL LUGAR
    % Calcular el ángulo al objetivo
    angle_to_goal = atan2(y_goal - y, x_goal - x);
    
    % Calcular la diferencia de ángulos más corta
    angle_diff = angle_to_goal - theta;
    
    % Normalizar la diferencia de ángulos para estar en el rango [-pi, pi]
    angle_diff = mod(angle_diff + pi, 2*pi) - pi;
    
    % Calcular el tiempo necesario para rotar
    t_rot = abs(angle_diff) / w;  % Tiempo en segundos para llegar al ángulo
    
    % Actualizar la orientación del robot
    theta = theta + angle_diff;
    
    % Registrar la trayectoria
    x1(end+1) = x;
    y1(end+1) = y;
    phi(end+1) = theta;
    hx(end+1) = x;
    hy(end+1) = y;

    % Calcular la distancia al objetivo
    dist = sqrt((x_goal - x)^2 + (y_goal - y)^2);
    
    % Calcular el tiempo necesario para moverse hasta el objetivo
    t_trans = dist / v;  % Tiempo en segundos para cubrir la distancia
    
   % Movimiento lineal paso a paso
    n_steps = ceil(t_trans / ts); % Número de pasos
    for j = 1:n_steps
        x = x + v * cos(theta) * ts;
        y = y + v * sin(theta) * ts;
    
        % Registrar trayectoria paso a paso
        x1(end+1) = x;
        y1(end+1) = y;
        phi(end+1) = theta;
        hx(end+1) = x;
        hy(end+1) = y;
    end
end

% Tiempo total de simulación
k_global = length(x1);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SIMULACIÓN VIRTUAL %%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Configuración de escena
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
view([0 90]);

% Límites automáticos
xmin_lim = min(x_vec) - 2;
xmax_lim = max(x_vec) + 2;
ymin_lim = min(y_vec) - 2;
ymax_lim = max(y_vec) + 2;
axis([xmin_lim xmax_lim ymin_lim ymax_lim 0 1]);

% Gráfica inicial
scale = 4;
MobileRobot_5; 
H1 = MobilePlot_4(x1(1), y1(1), phi(1), scale); hold on;
H2 = plot3(hx(1), hy(1), 0, 'r', 'lineWidth', 2);
H3 = plot3(x_vec, y_vec, 0 * ones(size(x_vec)), 'bo', 'lineWidth', 2);
H4 = plot3(hx(1), hy(1), 0, 'go', 'lineWidth', 2);

% Animación
step = 1;
for k = 1:step:k_global
    delete(H1);
    delete(H2);
    H1 = MobilePlot_4(x1(k), y1(k), phi(k), scale);
    H2 = plot3(hx(1:k), hy(1:k), zeros(1,k), 'r', 'lineWidth', 2);
    pause(ts);
end