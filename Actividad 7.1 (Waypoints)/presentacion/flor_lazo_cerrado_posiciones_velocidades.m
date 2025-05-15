%Limpieza de pantalla
clear all
close all
clc

sim_flag = true;

% Vectores de componentes X y Y de las coordenadas objetivos
x_vec = [4, 6, 8, 6, 4, 2, 0, 2, 4, 4, 2, 2, 0, 2, 0, 2, 2, 4, 6, 6, 8, 6, 8, 6, 6, 6, 4];
y_vec = [1, 1, 3, 3, 1, 3, 3, 1, 1, 5, 3, 5, 5, 7, 9, 9, 11, 9, 11, 9, 9, 7, 5, 5, 5, 3, 5];

%1 TIEMPO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tf=100;             % Tiempo de simulación en segundos (s)
ts=0.05;            % Tiempo de muestreo en segundos (s)
t=0:ts:tf;         % Vector de tiempo
N= length(t);      % Muestras


%2 CONDICIONES INICIALES %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Damos valores a nuestro punto inicial de posición y orientación
x1(1)=x_vec(1);  %Posición inicial eje x
y1(1)=y_vec(1);  %Posición inicial eje y
phi(1)=0; %Orientación inicial del robot 

%Igualamos el punto de control con las proyecciones X1 y Y1 por su
%coincidencia
hx(1)= x1(1);       % Posición del punto de control en el eje (X) metros (m)
hy(1)= y1(1);       % Posición del punto de control en el eje (Y) metros (m)

%3 TRAYECTORIA DESEADA %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

t_points = linspace(t(1), t(end), length(x_vec));

hxd = interp1(t_points, x_vec, t, 'linear'); % interpolación lineal
hyd = interp1(t_points, y_vec, t, 'linear');

%Velocidades de la trayectoria deseada
hxdp = gradient(hxd, ts);
hydp = gradient(hyd, ts);


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
    K=[3 0;...
       0 3];
    
    %d)Velocidades deseadas
    hdp=[hxdp(k);hydp(k)];

    %e)Ley de Control:Agregamos las velocidades deseadas
    qpRef= pinv(J)*(hdp + K*he);
    
    % VELOCIDADES
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
    % POSE
    x1(k+1)=x1(k)+ ts*xp1; % Integral numérica (método de Euler)
    y1(k+1)=y1(k)+ ts*yp1; % Integral numérica (método de Euler)

    % Posicion del robot con respecto al punto de control
    hx(k+1)=x1(k+1); 
    hy(k+1)=y1(k+1);
     

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SIMULACION VIRTUAL 3D %%%%%%%%%%%%%%%%%%%%%%%%%%%%
if sim_flag
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
    % Límites automáticos 
    xmin_lim = min(x_vec) - 2;
    xmax_lim = max(x_vec) + 2;
    ymin_lim = min(y_vec) - 2;
    ymax_lim = max(y_vec) + 2;
    axis([xmin_lim xmax_lim ymin_lim ymax_lim 0 1]);
    
    % b) Graficar robots en la posicion inicial
    scale = 4;
    MobileRobot_5;
    H1=MobilePlot_4(x1(1),y1(1),phi(1),scale);hold on;
    
    plot(x_vec, y_vec, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
    
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
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Graficas %%%%%%%%%%%%%%%%%%%%%%%%%%%%
graph=figure;  % Crear figura (Escena)
%set(graph,'position',sizeScreen); % Congigurar tamaño de la figura

figure;
subplot(211)
plot(t,v,'b','LineWidth',2),grid('on'),xlabel('Tiempo [s]'),ylabel('m/s'),legend('Velocidad Lineal (v)');
subplot(212)
plot(t,w,'g','LineWidth',2),grid('on'),xlabel('Tiempo [s]'),ylabel('[rad/s]'),legend('Velocidad Angular (w)');
%subplot(313)
%plot(t,Error,'r','LineWidth',2),grid('on'),xlabel('Tiempo [s]'),ylabel('[metros]'),legend('Error de posición (m)');

figure;
subplot(3,1,1)
plot(t,x1(1:end-1), 'g', 'LineWidth', 2), grid('on'),xlabel('Tiempo [s]'),ylabel('m'),legend('pose:xp');

subplot(3,1,2)
plot(t,y1(1:end-1), 'b', 'LineWidth', 2), grid('on'),xlabel('Tiempo [s]'),ylabel('m'),legend('pose:yp');

subplot(3,1,3)
plot(t,rad2deg(w), 'r', 'LineWidth', 2), grid('on'),xlabel('Tiempo [s]'),ylabel('Grados (°)'),legend('pose:thp');

figure;
plot(x1,y1)