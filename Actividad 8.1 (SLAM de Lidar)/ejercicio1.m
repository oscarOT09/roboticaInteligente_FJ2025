%% EXAMPLE: Differential Drive Path Following
% In this example, a differential drive robot navigates a set of waypoints 
% using the Pure Pursuit algorithm while avoiding obstacles using the
% Vector Field Histogram (VFH) algorithm.
% 
% Copyright 2019 The MathWorks, Inc.

clear all

op = 4; % 1: trayectoria 1, 2: trayectoria 2, 3: trayectoria 3, 4: trayectoria 4 para complexMap
ftimes_array = [38, 70, 130,275];
%% Simulation setup
% Define Vehicle
R = 0.05;                        % Wheel radius [m]
L = 0.18;                        % Wheelbase [m]
dd = DifferentialDrive(R,L);

% Sample time and time array
sampleTime = 0.1;              % Sample time [s]
tVec = 0:sampleTime:ftimes_array(op);        % Time array

% Load map

%complexMap       41x52                2132  logical              
%emptyMap         26x27                 702  logical              
%simpleMap        26x27                 702  logical              
%ternaryMap      501x501            2008008  double  

close all
load complexMap

% Create lidar sensor
lidar = LidarSensor;
lidar.sensorOffset = [0,0];
lidar.scanAngles = linspace(-pi,pi,360);%51
lidar.maxRange = 2;%5

% Create visualizer
viz = Visualizer2D;
viz.hasWaypoints = true;
viz.mapName = 'map';
attachLidarSensor(viz,lidar);

%% Path planning and following


if op == 1
    waypoints = [
                 4 2;
                 4 6;
                 9 6;
                 9 2
                 ];
elseif op == 2
    waypoints = [
                 3 3;
                 3 7;
                 5 9;
                 10 9;
                 8 7;
                 10 4;
                 8 3;
                 ];
elseif op == 3
    waypoints = [
                2 2;
                2 5;
                2 8;
                5 11;
                2 11;
                5 8;
                8 8;
                11 8;
                11 11;
                8 11;
                8 5;
                11 5;
                8 2;
                11 2
                ];
elseif op == 4
    waypoints = [
                 5 18;
                 5,14;
                 5, 9;
                 5, 5;
                 11,5;
                 17,5;
                 17,9;
                 23,9;
                 23,14;
                 23,18;
                 17,18;
                 11,18
                 
    ];
end

% Initial conditions
initPose = [waypoints(1,1);waypoints(1,2);0];            % Initial pose (x y theta)
pose = zeros(3,numel(tVec));   % Pose matrix
pose(:,1) = initPose;

% Pure Pursuit Controller
controller = controllerPurePursuit;
controller.Waypoints = waypoints;
controller.LookaheadDistance = 0.5;%0.5
controller.DesiredLinearVelocity = 0.37; %0.75
controller.MaxAngularVelocity = 5;

% Vector Field Histogram (VFH) for obstacle avoidance
vfh = controllerVFH;
vfh.DistanceLimits = [0.1 2]; %0.05 3
vfh.NumAngularSectors = 36; %36
vfh.HistogramThresholds = [0.5 2]; % 5y 10
vfh.RobotRadius = L/2;
vfh.SafetyDistance = 0.25;
vfh.MinTurningRadius = 0.75;%0.25

%% Simulation loop
r = rateControl(1/sampleTime);
for idx = 2:numel(tVec) 
    
    % Get the sensor readings
    curPose = pose(:,idx-1);
    ranges = lidar(curPose);
        
    % Run the path following and obstacle avoidance algorithms
    [vRef,wRef,lookAheadPt] = controller(curPose);
    targetDir = atan2(lookAheadPt(2)-curPose(2),lookAheadPt(1)-curPose(1)) - curPose(3);
    steerDir = vfh(ranges,lidar.scanAngles,targetDir);    
    if ~isnan(steerDir) && abs(steerDir-targetDir) > 0.1
        wRef = 0.5*steerDir;
    end
    
    % Control the robot
    velB = [vRef;0;wRef];                   % Body velocities [vx;vy;w]
    vel = bodyToWorld(velB,curPose);  % Convert from body to world
    
    % Perform forward discrete integration step
    pose(:,idx) = curPose + vel*sampleTime; 
    
    % Update visualization
    viz(pose(:,idx),waypoints,ranges)
    waitfor(r);
end

%plot(pose(1,:), pose(2,:))