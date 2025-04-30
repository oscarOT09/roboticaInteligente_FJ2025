%Limpieza de pantalla
clear all
close all
clc

% Selección de trayectorias
op = 3;

k_array = [3, 5, 3];

if op==1
    % Definición de los vértices del hexágono
    n = 7;
    theta1 = linspace(-pi, pi, n);
    r1 = ones(1,n) * 10; % 10=Radio de la figura
    
    vertices_x = r1.*cos(theta1);
    vertices_y = r1.*sin(theta1);
    
    % Interpolación de los vértices para cada instante de tiempo
    tf = 40;    % Tiempo final
    ts = 0.05;  % Tiempo de muestreo
    t = 0:ts:tf;
    N = length(t);
    
    t_vertices = linspace(0, tf, n); % tiempos para cada vértice
    
    % Interpolación para cada componente
    hxd = interp1(t_vertices, vertices_x, t, 'linear');
    hyd = interp1(t_vertices, vertices_y, t, 'linear');
    
    % Velocidades deseadas (derivadas numéricas aproximadas)
    hxdp = [diff(hxd) 0]/ts; % La última derivada se añade como 0
    hydp = [diff(hyd) 0]/ts;

    % Valores iniciales
    x1(1)=-10;  %Posición inicial eje x
    y1(1)=0;  %Posición inicial eje y
    phi(1)=-pi/4; %Orientación inicial del robot 

elseif op == 2
    tf = 40;    % Tiempo final
    ts = 0.1;  % Tiempo de muestreo
    t = 0:ts:tf;
    N = length(t); 

    s = linspace(0, 2*pi, N); % Parametrización en el mismo número de muestras
    
    % Componentes de las trayectorias deseadas
    hxd = 12*sin(s) - 4*sin(3*s);
    hyd = 13*cos(s) - 5*cos(2*s) - 2*cos(3*s)-4*cos(16);
    
    % Velocidades deseadas (derivadas numéricas aproximadas)
    hxdp = [diff(hxd) 0]/ts; % La última derivada se añade como 0
    hydp = [diff(hyd) 0]/ts;

    % Valores iniciales
    x1(1)=0;  %Posición inicial eje x
    y1(1)=10;  %Posición inicial eje y
    phi(1)=pi/4; %Orientación inicial del robot 
elseif op==3
    % Definir los vértices de la figura
    n = 100;
    theta1 = linspace(0, pi, n);
    r1 = 10 * cos(8*theta1); % 8 = radio de la figura
    
    vertices_x = r1.*cos(theta1);
    vertices_y = r1.*sin(theta1);
    
    % Interpolación de los vértices para cada instante de tiempo
    tf = 40;    % Tiempo final
    ts = 0.1;  % Tiempo de muestreo
    t = 0:ts:tf;
    N = length(t);
    
    t_vertices = linspace(0, tf, n); % tiempos para cada vértice
    
    % Interpolación para cada componenete de la trayectoria deseada
    hxd = interp1(t_vertices, vertices_x, t, 'linear');
    hyd = interp1(t_vertices, vertices_y, t, 'linear');
    
    % Velocidades deseadas (derivadas numéricas aproximadas)
    hxdp = [diff(hxd) 0]/ts; % La última derivada se añade como 0
    hydp = [diff(hyd) 0]/ts;

    % Valores iniciales
    x1(1)=10;  %Posición inicial eje x
    y1(1)=0;  %Posición inicial eje y
    phi(1)=pi/2; %Orientación inicial del robot 
end

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
    K=[k_array(op) 0;...
       0 k_array(op)];
    
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

