% Limpieza de pantalla
clear all
close all
clc


% Vectores de componentes X y Y de las coordenadas objetivos
x_vec = [4, 6, 8, 6, 4, 2, 0, 2, 4, 4, 2, 2, 0, 2, 0, 2, 2, 4, 6, 6, 8, 6, 8, 6, 6, 6, 4];
y_vec = [1, 1, 3, 3, 1, 3, 3, 1, 1, 5, 3, 5, 5, 7, 9, 9, 11, 9, 11, 9, 9, 7, 5, 5, 5, 3, 5];

%%%%%% COntron Lazo Cerrado %%%%%%%%%%%%%
sim_flag = true;
% Tiempo de simulación y muestreo
tf = 25;          % Tiempo total (s) largo porque iremos por muchos puntos
ts = 0.05;         % Tiempo de muestreo (s)
t = 0:ts:tf;
N = length(t);

% Condiciones iniciales
x1(1) = x_vec(1);
y1(1) = y_vec(1);
phi(1) = 0;

% Punto de control
hx(1) = x1(1);
hy(1) = y1(1);

% Inicializamos vectores de velocidad
v = zeros(1, N);
w = zeros(1, N);
Error = zeros(1, N);

k_global = 1; % índice global para recorrer todo el vector de tiempo

% Bucle principal para recorrer todos los puntos de x_vec e y_vec
for i = 1:length(x_vec)

    hxd = x_vec(i);
    hyd = y_vec(i);

    while k_global < N
        % Errores de posición
        hxe(k_global) = hxd - hx(k_global);
        hye(k_global) = hyd - hy(k_global);
        Error(k_global) = sqrt(hxe(k_global)^2 + hye(k_global)^2);

        % Condición de llegada al punto
        if Error(k_global) < 0.17
            break;
        end

        % Jacobiano
        J = [cos(phi(k_global)) -sin(phi(k_global));
             sin(phi(k_global))  cos(phi(k_global))];

        % Ganancias
        K = [7 0;
             0 7];

        % Ley de control
        he = [hxe(k_global); hye(k_global)];
        qpRef = pinv(J) * K * he;

        % VELOCIDADES
        v(k_global) = qpRef(1);
        w(k_global) = qpRef(2);

        % Actualización del estado
        % POSE
        phi(k_global + 1) = phi(k_global) + w(k_global) * ts;
        xp1 = v(k_global) * cos(phi(k_global));
        yp1 = v(k_global) * sin(phi(k_global));
        x1(k_global + 1) = x1(k_global) + xp1 * ts;
        y1(k_global + 1) = y1(k_global) + yp1 * ts;
        hx(k_global + 1) = x1(k_global + 1);
        hy(k_global + 1) = y1(k_global + 1);

        k_global = k_global + 1;
    end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SIMULACIÓN VIRTUAL %%%%%%%%%%%%%%%%%%%%%%%%%%%%
if sim_flag
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
    H5 = plot3(x_vec, y_vec, zeros(size(x_vec)),'g','lineWidth',2);
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
end
figure;
subplot(211)
plot(t,v,'b','LineWidth',2),grid('on'),xlabel('Tiempo [s]'),ylabel('m/s'),legend('u');
subplot(212)
plot(t,w,'r','LineWidth',2),grid('on'),xlabel('Tiempo [s]'),ylabel('[rad/s]'),legend('w');

figure;
subplot(3,1,1)
plot(linspace(0,tf,length(x1)),x1, 'g', 'LineWidth', 2), grid('on'),xlabel('Tiempo [s]'),ylabel('m'),legend('pose:xp');

subplot(3,1,2)
plot(linspace(0,tf,length(x1)),y1, 'b', 'LineWidth', 2), grid('on'),xlabel('Tiempo [s]'),ylabel('m'),legend('pose:yp');

subplot(3,1,3)
plot(t,rad2deg(w), 'r', 'LineWidth', 2), grid('on'),xlabel('Tiempo [s]'),ylabel('Grados (°)'),legend('pose:thp');
