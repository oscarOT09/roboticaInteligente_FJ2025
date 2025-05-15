%% Define vehicle
R = 0.05; % Wheel Radius [m]
L = 0.18; % Wheel base [m]
dd = DifferentialDrive(R,L);

op = 2;
tf_array = [80,65,50];

%% Simulation parameters
sampleTime = 0.05; % Sample time [s]
tVec = 0:sampleTime:tf_array(op); % Time array
pose_th = [pi, 0, pi];

% Define waypoints
%waypoints = [0,0; 2,2; 4,2; 2,4; 0.5,3];
if op == 1
    waypoints = [12,6; 11,6; 9,8; 7,12; 7,11; 6,11; 5,12; 5,10; 3,10; 3,9; 1,9; 1,8; 2,6; 5,6; 6,5; 6,4; 6,2; 7,0; 12,0; 12,6];
elseif op == 2
    waypoints = [4,1; 6,1; 8,3; 6,3; 4,1; 2,3; 0,3; 2,1; 4,1; 4,5; 2,3; 2,5; 0,5; 2,7; 0,9; 2,9; 2,11; 4,9; 6,11; 6,9; 8,9; 6,7; 8,5; 6,5; 6,5; 6,3; 4,5];
elseif op == 3
    waypoints = [10,9; 8,11; 6,11; 4,9; 10,9; 7,5; 6,6; 7,5; 8,5; 7,5; 7.8,6.2; 9,5; 9,3; 7,1; 5,1; 3,3; 3,5; 5,7; 7,7; 9,5];
end

initPose = [waypoints(1,1);waypoints(1,2); pose_th(op)]; % Initial pose (x, y, theta)
pose = zeros(3, numel(tVec)); % Pose matrix
pose(:,1) = initPose;

% Create visualizer
viz = Visualizer2D;
viz.hasWaypoints = true;

%% Pure Pursuit Controller
controller = controllerPurePursuit;
controller.Waypoints = waypoints;
controller.LookaheadDistance = 0.4;
controller.DesiredLinearVelocity = 1;
controller.MaxAngularVelocity = 3;
wL_array = [];
wR_array = [];

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
    velB = [v;0;w]; % Body velocities [vx;vy;w]
    vel = bodyToWorld(velB, pose(:,idx-1)); % Convert from body to world

    % Perform forward discrete integration step
    pose(:, idx) = pose(:, idx-1) + vel*sampleTime;

    % Update visualization
    viz(pose(:,idx), waypoints)
    waitfor(r);
end