% Sample script for simulating the Pioneer moving
% to a line and following that line in VREP via ROS
% Eric Jones 10/12/2018


clear
clc
close all

disp('Please wait while the simulation runs ...');


%==========================================================================
% SIMULATION PARAMETERS
%==========================================================================

% Simulation time
t0 = 0; tf = 25; % Initial and final simulation time [s]
T = 0.05;  % Sampling time [s]
tsteps = floor((tf-t0)/T); % number of time steps
dt = T*(0:tsteps)'; % Discrete time vector (include time t0 to plot initial state too) 
%==========================================================================
Kp = 0.5;    % formerly 0.5 proportional gain on Position
Kg = 1.25;    % formerly 1.0 proportional gain on Gamma
%==========================================================================
l = 0.381; % [m]
r = 0.0925; % [m] radius
%==========================================================================


%==========================================================================
% INITIALIZATION
%==========================================================================

%ROS Setup
nodeName = 'Pioneer';
rosCoreIP = '192.168.1.97';
node = robotics.ros.Node(nodeName, rosCoreIP);

posesub = rossubscriber('/robotsPose');
[vpub,msg] = rospublisher('/desiredVel','geometry_msgs/Vector3');

%==========================================================================

xInit = 0; yInit = -0.5; thetaInit = pi/2;  % x,y,theta  ==> q(0) in [m]

P1x = 0.3; %Reference P1x in [m]
P1y = 0.3; %Reference P1y in [m]
P2x = 1.8; %Reference P2x in [m] 
P2y = 1.8; %Reference P2y in [m]

DelXPoint = (P2y - P1y);
DelYPoint = (P2x - P1x);

m = (DelYPoint)/(DelXPoint); %Slope between P1 and P2
a = m; %Linear State Equation Variables
b = -1;
c = 0;

%==========================================================================
% REFERENCE MODEL
%==========================================================================

x(1) = xInit; %Initializing robot's first pose
y(1) = yInit;
th(1) = thetaInit;

for k = 1:tsteps+1 %Iterating over tsteps time
    
    %Receiving robots current pose
    posedata = receive(posesub,10);
    position(1) = posedata.X;
    position(2) = posedata.Y;
    orientation = posedata.Z;
    %==========================================================================
    
    %Robot's iterated pose
    x(k+1) = position(1); 
    y(k+1) = position(2);
    th(k+1) = orientation;

    d = ((a*x(k))+(b*y(k))+c)/(sqrt(a^2 + b^2)); %getting updated orthogonal distance

    dTheta = atan2(-a,b)-th(k); %getting updated slope
    dTheta = mod(dTheta, 2*pi); %used in limiting the values for dTheta

    %==========================================================================
    % LIMITING dTheta BETWEEN PI AND -PI %
    %==========================================================================
        if (dTheta > pi)
            dTheta = (dTheta - 2*pi);
        end
        if (dTheta < -pi)
            dTheta = (dTheta + 2*pi);
        end

    %==========================================================================
    gam = (-Kp*d) + (Kg *(dTheta)); %Applying gains as weights to gamma
    gam = sign(gam)*min(80*pi/180,abs(gam)); %Setting gamma between 80,-80 [deg]

    vk = 0.25; %Initializing (NOTE: vk is always constant)
    wk = (vk/l).*tan(gam); %update angular speed
    %==========================================================================
    nuR = (2*vk+l*wk)/2; % [m/s] robot's right wheel's linear speed
    nuL = (2*vk-l*wk)/2; % [m/s] robot's left wheel's linear speed 

    msg.X = nuR;
    msg.Y = nuL;
    msg.Z = 0;
    
    send(vpub,msg);
    pause(T); %Function iterates relative to sampling time

end

figure(1) %plotting x-y positions from VREP environment
plot([-2,2],[-2,2]);
hold on
plot(x,y)
title('Plot of Real-Time X-Y Position');
xlabel('X Position');
ylabel('Y Position');

msg.X = 0;
msg.Y = 0;
msg.Z = 0;
send(vpub,msg);
%==========================================================================


disp('... done.');



