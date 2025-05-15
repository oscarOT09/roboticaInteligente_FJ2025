clc
clear all
close all
%% Define vehicle
R = 0.05; % Wheel Radius [m]
L = 0.18; % Wheel base [m]
dd = DifferentialDrive(R,L);

op = 2;

%% Simulation parameters
sampleTime = 0.05; % Sample time [s]
tVec = 0:sampleTime:119; % Time array

% Define waypoints
waypoints = [4,1; 6,1; 8,3; 6,3; 4,1; 2,3; 0,3; 2,1; 4,1; 4,5; 2,3; 2,5; 0,5; 2,7; 0,9; 2,9; 2,11; 4,9; 6,11; 6,9; 8,9; 6,7; 8,5; 6,5; 6,5; 6,3; 4,5];

initPose = [waypoints(1,1);waypoints(1,2); 0]; % Initial pose (x, y, theta)
pose = zeros(3, numel(tVec)); % Pose matrix
pose(:,1) = initPose;

% Create visualizer
viz = Visualizer2D;
viz.hasWaypoints = true;

%% Pure Pursuit Controller
controller = controllerPurePursuit;
controller.Waypoints = waypoints;
controller.LookaheadDistance = 0.3;
controller.DesiredLinearVelocity = 0.5;
controller.MaxAngularVelocity = 5;
wL_array = [];
wR_array = [];
v_array = [];
w_array = [];

%% Simulation Loop
close all
r = rateControl(1/sampleTime);
for idx = 2:numel(tVec)
    % Run the Puse Pursuit controller and convert output to wheel speeds
    [vRef, wRef] = controller(pose(:,idx-1));
    [wL, wR] = inverseKinematics(dd, vRef, wRef);
    
    wL_array = [wL_array, wL];
    wR_array = [wR_array, wR];

    % Compute the velocities
    [v, w] = forwardKinematics(dd, wL, wR);
    v_array = [v_array, v];
    w_array = [w_array, w];
    velB = [v;0;w]; % Body velocities [vx;vy;w]
    vel = bodyToWorld(velB, pose(:,idx-1)); % Convert from body to world

    % Perform forward discrete integration step
    pose(:, idx) = pose(:, idx-1) + vel*sampleTime;

    % Update visualization
    viz(pose(:,idx), waypoints)
    waitfor(r);
end
figure;
subplot(211)
plot(tVec(1:end-1),v_array,'b','LineWidth',2),grid('on'),xlabel('Tiempo [s]'),ylabel('m/s'),legend('u');
subplot(212)
plot(tVec(1:end-1),w_array,'r','LineWidth',2),grid('on'),xlabel('Tiempo [s]'),ylabel('[rad/s]'),legend('w');

figure;
subplot(3,1,1)
plot(tVec,pose(1,:), 'g', 'LineWidth', 2), grid('on'),xlabel('Tiempo [s]'),ylabel('m'),legend('pose:xp');

subplot(3,1,2)
plot(tVec,pose(2,:), 'b', 'LineWidth', 2), grid('on'),xlabel('Tiempo [s]'),ylabel('m'),legend('pose:yp');

subplot(3,1,3)
plot(tVec,pose(3,:), 'r', 'LineWidth', 2), grid('on'),xlabel('Tiempo [s]'),ylabel('Grados (°)'),legend('pose:thp');