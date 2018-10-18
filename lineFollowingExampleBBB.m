% Sample script for simulating the modified BBBlue moving
% to a line and following that line
% Eric Jones 10/12/2018

function lineFollowingExampleBBB
clear
close all
clc

disp('Please wait while the simulation runs ...');

vrep=remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

%==========================================================================
% SIMULATION PARAMETERS
%==========================================================================

% Simulation time
t0 = 0; tf = 30; % Initial and final simulation time [s]
T = 0.05;  % Sampling time [s]
tsteps = floor((tf-t0)/T); % number of time steps
dt = T*(0:tsteps)'; % Discrete time vector (include time t0 to plot initial state too) 
%==========================================================================
Kp = 0.5; % formerly 0.5 proportional gain on Position
Kg = 1;    %              proportional gain on Gamma
%==========================================================================
l = 0.16; % [m]
r = 0.06; % [m] radius
%==========================================================================

%Get the Pioneer robot handles
[rtn,robot(1,1)] = vrep.simxGetObjectHandle(clientID,'BBBTest', vrep.simx_opmode_blocking);
[rtn,robot(1,2)] = vrep.simxGetObjectHandle(clientID,'RightMotor', vrep.simx_opmode_blocking);
[rtn,robot(1,3)] = vrep.simxGetObjectHandle(clientID,'LeftMotor', vrep.simx_opmode_blocking);

% Stream robot pose (position and orientation)
vrep.simxGetObjectPosition(clientID,robot(1,1),-1,vrep.simx_opmode_streaming);
vrep.simxGetObjectOrientation(clientID,robot(1,1),-1,vrep.simx_opmode_streaming);

%==========================================================================
% INITIALIZATION
%==========================================================================

xInit = 0; yInit = -0.5; thetaInit = 0;  % x,y,theta  ==> q(0) in [m]

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

[~,position] = vrep.simxGetObjectPosition(clientID,robot(1,1),-1,vrep.simx_opmode_buffer); %Get postition
[~,orientation] = vrep.simxGetObjectOrientation(clientID,robot(1,1),-1,vrep.simx_opmode_buffer); %Get orientation

%==========================================================================
x(k+1) = position(1); %Iterated pose
y(k+1) = position(2);
th(k+1) = orientation(3);

d = ((a*x(k))+(b*y(k))+c)/(sqrt(a^2 + b^2)) %getting updated orthogonal distance

dTheta = atan2(a,b)-th(k); %getting updated slope
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
wk = (vk/.381).*tan(gam); %update angular speed
%==========================================================================
nuR = (2*vk+l*wk)/2; % [m/s] robot's right wheel's linear speed
nuL = (2*vk-l*wk)/2; % [m/s] robot's left wheel's linear speed 

vrep.simxPauseCommunication(clientID,1); %Moving the robot
vrep.simxSetJointTargetVelocity(clientID,robot(1,2),nuR,vrep.simx_opmode_oneshot); 
vrep.simxSetJointTargetVelocity(clientID,robot(1,3),nuL,vrep.simx_opmode_oneshot); 
vrep.simxPauseCommunication(clientID,0);

pause(T); %Function iterates relative to sampling time

end

figure(1) %plotting x-y positions from VREP environment
plot([-2,2],[-2,2]);
hold on
plot(x,y)
title('Plot of Real-Time X-Y Position');
xlabel('X Position');
ylabel('Y Position');

vrep.simxPauseCommunication(clientID,1); %Stopping the robot
vrep.simxSetJointTargetVelocity(clientID,robot(1,2),0,vrep.simx_opmode_oneshot); 
vrep.simxSetJointTargetVelocity(clientID,robot(1,3),0,vrep.simx_opmode_oneshot); 
vrep.simxPauseCommunication(clientID,0);

%==========================================================================
vrep.simxGetPingTime(clientID);
vrep.simxFinish(clientID);
vrep.delete();

disp('... done.');
end



