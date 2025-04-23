% Limpieza de pantalla
clear all
close all
clc

op = 2;
% Vectores de componentes X y Y de las coordenadas objetivos
if op == 1
    x_vec = [1, 5, 5, 3, 3, 4, 6, 6, 9, 9, 8, 8, 9, 11, 13, 15, 16, 16, 14, 10,  8, 8, 6, 4, 2,  2,  3,  5,  5,  3,  0, 0, 0];
    y_vec = [0, 0, 2, 2, 3, 4, 4, 0, 0, 2, 2, 5, 4,  3,  3,  4,  5, 11,  9,  9, 11, 7, 9, 9, 7, 10, 10, 12, 13, 13, 10, 6, 0];
elseif op == 2
    x_vec = [1, 2, 3, 3, 4.8, 6, 8, 10, 12, 13, 13, 15, 19, 22, 24, 26, 24.8,22.8,22,18.8, 17,16,15, 15,13,11, 8.6, 8, 6.8, 5, 1, 0, -4, -6, -8, -10, -11, -12, -13, -14, -17, -18, -21.8, -24, -24.8, -26.2, -25, -23, -21, -19, -14.8, -12, -10, -7, -4, -1, -1, -2, 0, 1];
    y_vec = [0, 0, 1, 3.3, -1, -2, 0, 4, 8, 13, 16, 15, 15, 15, 15, 17, 14, 10, 6, 5, 3,1, -2, -7, -8, -10, -12, -13.8, -18, -15.8, -14, -14, -15, -17, -15, -13, -12, -12, -12, -13, -15, -15, -13, -12, -12, -13, -11, -8, -5, 0, -2, -4, -5, -6, -6, -6, -5, 1, -1, 0];
elseif op == 3
    x_vec = [-0.8, 0, 0.6, 2, 3.2, 4, 3.2, 2.6, 4, 4, 2.6, 2, 0.6, 0, -0.6, -2, -2.6, -4, -4, -2.6, -3.2, -4, -3.2, -2, -0.6, 0, 0.8];
    y_vec = [6.8, 6, 5.2, 5.6, 5.6, 5, 2.8, 2, 0.8, 0, -0.6, 0, 0.8, 0, 0.8, 0, -0.6, 0, 0.8, 2, 2.8, 4, 5.6, 5.6, 5.2, 6, 6.8];
end

% Tiempo de simulación y muestreo
tf = 10*length(x_vec);          % Tiempo total (s) largo porque iremos por muchos puntos
ts = 0.1;         % Tiempo de muestreo (s)
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

        % Jacobiana
        J = [cos(phi(k_global)) -sin(phi(k_global));
             sin(phi(k_global))  cos(phi(k_global))];

        % Ganancias
        K = [1.5 0;
             0 1.5];

        % Ley de control
        he = [hxe(k_global); hye(k_global)];
        qpRef = pinv(J) * K * he;
        v(k_global) = qpRef(1);
        w(k_global) = qpRef(2);

        % Actualización del estado
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

% Límites automáticos (puedes ajustar manualmente si quieres)
xmin_lim = min(x_vec) - 2;
xmax_lim = max(x_vec) + 2;
ymin_lim = min(y_vec) - 2;
ymax_lim = max(y_vec) + 2;
axis([xmin_lim xmax_lim ymin_lim ymax_lim 0 1]);

% Gráfica inicial
scale = 4;
MobileRobot_5;  % Suponiendo que tienes esta función
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
%{
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Gráficas %%%%%%%%%%%%%%%%%%%%%%%%%%%%
graph = figure;
set(graph,'position',sizeScreen);
subplot(311)
plot(t(1:k_global), v(1:k_global), 'b', 'LineWidth', 2), grid on
xlabel('Tiempo [s]'), ylabel('m/s'), legend('Velocidad Lineal (v)');
subplot(312)
plot(t(1:k_global), w(1:k_global), 'g', 'LineWidth', 2), grid on
xlabel('Tiempo [s]'), ylabel('[rad/s]'), legend('Velocidad Angular (w)');
subplot(313)
plot(t(1:k_global), Error(1:k_global), 'r', 'LineWidth', 2), grid on
xlabel('Tiempo [s]'), ylabel('[m]'), legend('Error de posición');
%}