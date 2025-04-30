%Limpieza de pantalla
clear all
close all
clc


% Selección de trayectoria
i = 8;

% Tiempos de muestreo
ts_array = [0.05, 0.05, 0.02, 0.01, 0.03, 0.03, 0.05, 0.03];
% Direcciones iniciales
ang_array = [pi/2, pi, pi/2, 0, pi/4, pi/2, 0, pi];
% Tiempos de ejecución
time_array = [31.5, 30, 6.3, 7, 6.3, 6.35, 10, 6.2];
% Ganancias del controlador
k_array = [10, 12, 7, 10, 20, 18, 10, 12];

% Parametrización de tiempo
tf = time_array(i);    % Tiempo final
ts = ts_array(i);  % Tiempo de muestreo
t = 0:ts:tf;
N = length(t);

% Funciones de las trayectorias
x_array = { ...
    2*cos(0.2*t), ...
    t - 3*sin(t), ...
    3*cos(t) - cos(3*t), ...
    cos(t) + 0.5*cos(7*t) + (1/3)*sin(17*t), ...
    17*cos(t) + 7*cos(17 + 7*t), ...
    2*cos(t), ...
    5*t - 4*sin(t), ...
    4*cos(t) + cos(4*t) ...
    };

y_array = { ...
    2*sin(0.4*t), ...
    4 - 3*cos(t), ...
    4*sin(3*t), ...
    sin(t) + 0.5*sin(7*t) + (1/3)*cos(17*t), ...
    17*sin(t) - 7*sin(17 + 7*t), ...
    2*sin(t), ...
    5*t - 4*cos(t), ...
    4*sin(t) - sin(4*t) ...
    };

% Trayectoria deseada
hxd = x_array{i};
hyd = y_array{i};

% Calculo de velocidades deseadas
hxdp = gradient(hxd, ts);
hydp = gradient(hyd, ts);

x1(1)=hxd(1);  %Posición inicial eje x
y1(1)=hyd(1);  %Posición inicial eje y
phi(1)=ang_array(i); %Orientación inicial del robot

%Igualamos el punto de control con las proyecciones X1 y Y1 por su
%coincidencia
hx(1)= x1(1);       % Posición del punto de control en el eje (X) metros (m)
hy(1)= y1(1);       % Posición del punto de control en el eje (Y) metros (m)



%4 CONTROL, BUCLE DE SIMULACION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for k=1:N 

    %a)Errores de control (Aqui la posición deseada ya no es constante,
    % varia con el tiempo)
    hxe(k)=hxd(k)-hx(k);
    hye(k)=hyd(k)-hy(k);
    
    %Matriz de error
    he= [hxe(k);hye(k)];
    %Magnitud del error de posición
    Error(k)= sqrt(hxe(k)^2 +hye(k)^2);

    %b)Matriz Jacobiana
    J=[cos(phi(k)) -sin(phi(k));... %Matriz de rotación en 2D
       sin(phi(k)) cos(phi(k))];

    %c)Matriz de Ganancias
    K=[k_array(i) 0;...
       0 k_array(i)];
    
    %d)Velocidades deseadas
    hdp=[hxdp(k);hydp(k)];

    %e)Ley de Control:Agregamos las velocidades deseadas
    qpRef= pinv(J)*(hdp + K*he);

    v(k)= qpRef(1);   %Velocidad lineal de entrada al robot 
    w(k)= qpRef(2);   %Velocidad angular de entrada al robot 


%5 APLICACIÓN DE CONTROL AL ROBOT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %Aplico la integral a la velocidad angular para obtener el angulo "phi" de la orientación
    phi(k+1)=phi(k)+w(k)*ts; % Integral numérica (método de Euler)
           
   %%%%%%%%%%%%%%%%%%%%% MODELO CINEMATICO %%%%%%%%%%%%%%%%%%%%%%%%%
    
    xp1=v(k)*cos(phi(k)); 
    yp1=v(k)*sin(phi(k));
 
    %Aplico la integral a la velocidad lineal para obtener las cordenadas
    %"x1" y "y1" de la posición
    x1(k+1)=x1(k)+ ts*xp1; % Integral numérica (método de Euler)
    y1(k+1)=y1(k)+ ts*yp1; % Integral numérica (método de Euler)

    % Posicion del robot con respecto al punto de control
    hx(k+1)=x1(k+1); 
    hy(k+1)=y1(k+1);
     

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SIMULACION VIRTUAL 3D %%%%%%%%%%%%%%%%%%%%%%%%%%%%

% a) Configuracion de escena

scene=figure;  % Crear figura (Escena)
set(scene,'Color','white'); % Color del fondo de la escena
set(gca,'FontWeight','bold') ;% Negrilla en los ejes y etiquetas
sizeScreen=get(0,'ScreenSize'); % Retorna el tamaño de la pantalla del computador
set(scene,'position',sizeScreen); % Configurar tamaño de la figura
camlight('headlight'); % Luz para la escena
axis equal; % Establece la relación de aspecto para que las unidades de datos sean las mismas en todas las direcciones.
grid on; % Mostrar líneas de cuadrícula en los ejes
box on; % Mostrar contorno de ejes
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)'); % Etiqueta de los eje

view([-0.1 90]); % Orientacion de la figura

%%%
% Calcular los límites automáticos
padding = 2; % Margen extra que quieres darle

x_min = min(hxd) - padding;
x_max = max(hxd) + padding;
y_min = min(hyd) - padding;
y_max = max(hyd) + padding;

% Establecer los límites automáticamente
axis([x_min x_max y_min y_max 0 1]);
%%%

%axis([-20 20 -20 20 0 1]); % Ingresar limites minimos y maximos en los ejes x y z [minX maxX minY maxY minZ maxZ]

% b) Graficar robots en la posicion inicial
scale = 4;
MobileRobot_5;
H1=MobilePlot_4(x1(1),y1(1),phi(1),scale);hold on;

% c) Graficar Trayectorias
H2=plot3(hx(1),hy(1),0,'r','lineWidth',2);
H3=plot3(hxd,hyd,zeros(1,N),'g','lineWidth',2); %Grafico circulo en posición deseada
%H4=plot3(hx(1),hy(1),0,'go','lineWidth',2);%Grafico circulo en posición inicial
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
subplot(311)
plot(t,v,'b','LineWidth',2),grid('on'),xlabel('Tiempo [s]'),ylabel('m/s'),legend('Velocidad Lineal (v)');
subplot(312)
plot(t,w,'g','LineWidth',2),grid('on'),xlabel('Tiempo [s]'),ylabel('[rad/s]'),legend('Velocidad Angular (w)');
subplot(313)
plot(t,Error,'r','LineWidth',2),grid('on'),xlabel('Tiempo [s]'),ylabel('[metros]'),legend('Error de posición (m)');

