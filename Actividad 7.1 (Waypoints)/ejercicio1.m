%% Define vehicle
R = 0.05; % Wheel Radius [m]
L = 0.18; % Wheel base [m]
dd = DifferentialDrive(R,L);

op = 3;
tf_array = [80,55,35];

%% Simulation parameters
sampleTime = 0.05; % Sample time [s]
tVec = 0:sampleTime:tf_array(op); % Time array
pose_th = [pi, pi, 0];

% Define waypoints
%waypoints = [0,0; 2,2; 4,2; 2,4; 0.5,3];
if op == 1
    waypoints = [4,4;-10,8;7,-1;-7,-6;0,5;-3.4,0;1.7,-5;0,0];
elseif op == 2
    waypoints = [2,5;-5,3;-5,-2;2,-5;5,2;-3,2;-4,-4;4,-3];
elseif op == 3
    waypoints = [-3,4;3,3;1,-3;-1,-1;1,4;-3,-4;2,-1];
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