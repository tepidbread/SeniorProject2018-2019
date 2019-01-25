% Sample script for implementing line following on the
% BBBlue using ROS Linux and MATLAB ROS
% Eric Jones 1/7/2019

%function lineFollowingImplementation
clear
clc
close all

disp('Please wait while the simulation runs ...');

%==========================================================================
% SIMULATION PARAMETERS
%==========================================================================

% Simulation time
t0 = 0; tf = 10;    % Initial and final simulation time [s]
T = 0.005;           % Sampling time [s]
timeElapsed = 0;
%==========================================================================
Kp = 2;         % formerly 2 proportional gain on Position
Kg = 0.625;      % formerly 0.625 proportional gain on Gamma
vk = 0.15;       % Initializing constant velocity
%==========================================================================
l = 0.0825;       % [m] formerly         0.381 - Pioneer
r = 0.035;      % [m] radius formerly  0.0925 - Pioneer
%==========================================================================

%==========================================================================
% ROS INITIALIZATION
%==========================================================================

posesub = rossubscriber('/poseBBB','std_msgs/Float64MultiArray');
[vpub,msg] = rospublisher('/desiredSpeed','std_msgs/Float64MultiArray');
%==========================================================================

%Initialize pose variables
xInit = 0.90; yInit = 0.30; thetaInit = pi/2;  % x,y,theta  ==> q(0) in [m]

qInit = [xInit,yInit,thetaInit];
qk = qInit;  % read actual (follower) robot's pose

% Define some matrices to record pose and velocities
q = [0 0 0]; % state at time t  , n = 3 (state space dimension)
u = [0 0];  % m = 2

% % Generate reference trajectory (line) 
p1 = [1.0;1.06]; % [m] point 1
p2 = [0.0;0.06]; % [m] point 2
m = (p2(2)-p1(2))/(p2(1)-p1(1));  % slope

a = m; %Linear State Equation Variables
b = -1;
c = 0.06;

%==========================================================================
% REFERENCE MODEL
%==========================================================================

k = 1;      %Timing iterator
while (timeElapsed <= tf)
      
    tic;   %timing variable start
    
    %getting updated orthogonal distance
    d(k,1) = (a*qk(1) + b*qk(2) + c)/(sqrt(a^2+b^2));
    alpha_d = -Kp*d(k,1);
    thetad = atan2(-a,b);
    alpha_h = Kg*pi2pi(thetad-qk(3));
    
    gam = alpha_d + alpha_h;
    
    % limit steering angle
    %gam = sign(gam)*min(80*pi/180,abs(gam)); % maximum 80 [deg] turning

    wk = (vk/l).*tan(gam); %update angular speed
    nuR = (2*vk+l*wk)/2; % [m/s] robot's right wheel's linear speed
    nuL = (2*vk-l*wk)/2; % [m/s] robot's left wheel's linear speed 
    
    msg.Data = [nuL nuR]; % Sending nuL and nuR to the wheels
    
    send(vpub,msg);
    
    %======================================================================
    %Receiving robots current pose
    posedata = receive(posesub,10);
   
    %Robot's iterated pose
    position(1) = posedata.Data(1); 
    position(2) = posedata.Data(2);
    theta =  posedata.Data(3);
    
    q(k,:) = [position(1),position(2),theta];
    qk = q(k,:);  % read the follower robot's pose and reset
    
    u(k,:) = [vk wk];
    
    %timing  
    pause(T);
    toc;
    timeElapsed = timeElapsed + toc;
    time(1,k) = timeElapsed;
    
    k = k+1;   %Iterating looping variable
end

%Sending stop to the robot's wheels
msg.Data = [0 0]; % [m/s]
send(vpub,msg);   

Q_DT = [qInit;q]; % include intial state q0 too

actualPose = Q_DT;
distanceError = d; % error with respect to global coordinate systems


%Plot everything
generatePlots(length(time), time, actualPose , [], distanceError, u, p1, p2);
%==========================================================================

disp('... done.');


function generatePlots(Tn, t, actualStates, desiredStates, error, u, p1, p2)    
    close all; % close all opened figures
    % Tn = number of discrete time/path parameter points 
    % t = time/path parameter history, dimension = Tn x 1 
    % actualState = actual state of the system, dimension = Tn x n
    % desiredState = desired state of the system, dimension = Tn x n
    % error = desired state - actual state, dimension =  Tn x n
    % u = control inputs, dimension = Tn x m
         
 
    % Plot the robot's  velocities, 
    figure
    subplot(2,1,1)
    plot(t,u(:,1), 'k-','LineWidth', 1.5);   
    xlabel('Time [s]');
    ylabel('Linear speed [m/s]');        
    grid on
    
    subplot(2,1,2)
    plot(t(1:end),u(:,2), 'k--','LineWidth', 1.5);   
    xlabel('Time [s]');
    ylabel('Angular speed [rad/s]');
    grid on
    
    savefilename = ['OUT/controlInputs'];
    saveas(gcf, savefilename, 'fig');
    print('-depsc2', '-r300', [savefilename, '.eps']);
    
    % Create a movie of the simulation (path/trajectory)
    

    xmax = max(actualStates(:,1));
    xmin = min(actualStates(:,1));
    ymax = max(actualStates(:,2));
    ymin = min(actualStates(:,2));
          
    vid = VideoWriter('OUT/trajectory.avi');
    vid.Quality = 100;
    vid.FrameRate = 5;
    open(vid)
    
    fig=figure;
    clf reset;    
    
    for i = 1:Tn
        clf;
        box on;
        axis([xmin-0.5 xmax+0.5 ymin-0.5 ymax+0.5]);
        axis equal;
        axis manual;
        [Xa,Ya] = plot_DDMR(actualStates(i,:),axis(gca)); % 
        hold on;
        plot([p1(1) p2(1)], [p1(2) p2(2)]);
        hold on;
        %desired = plot(desiredStates(1:i,1),desiredStates(1:i,2),'LineWidth',1.5);
        %hold on
        actual = plot(actualStates(1:i,1),actualStates(1:i,2),'k--');
        fill(Xa,Ya,'r');
        hold off;
        xlabel('x [m]');
        ylabel('y [m]');
        F = getframe(fig);
        writeVideo(vid,F);          
    end
    [Xa,Ya] = plot_DDMR(actualStates(1,:),axis(gca)); % DDMR => Differential drive mobile robot
    grid on
    hold on
    plot(Xa,Ya);    
    hold on
%    legend([actual desired],'actual', 'desired');
    savefilename = 'OUT/trajectory';
    saveas(gcf, savefilename, 'fig');
    print('-depsc2', '-r300', [savefilename, '.eps']);
       
    close(vid);

    % Create the movie and save it as an AVI file
    % winopen('OUT/trajectory.avi')

    % plot error
    figure
    plot(t(1:end),error(:,1), 'k-','LineWidth', 1.5);   
    xlabel('Time [s]');
    ylabel('d [m]');        
    grid on  

    
%     plotxyxy(t,e,P,eP)    
    
%     RMSE = sqrt(mse(e));
%     s = strcat('RMSE =',num2str(RMSE), ' m');
%     text_xaxis = ceil(30);
%     text_yaxis = 2;
%     text(text_xaxis,text_yaxis,s);    
%     xlabel('Time [sec]');
%     ylabel('Pose error $e(t) = \|q(t)-q^d(t)\|$ [m]','Interpreter','Latex');    
%     %ylim([0 5]);
%     grid on
savefilename = 'OUT/error';
    saveas(gcf, savefilename, 'fig');
    print('-depsc2', '-r300', [savefilename, '.eps']);
    
end

function angle = pi2pi(angle)

% function angle = pi2pi(angle)
% Input: array of angles.
% Tim Bailey 2000 (Thank you)

angle = mod(angle, 2*pi);

i=find(angle>pi);
angle(i)=angle(i)-2*pi;

i=find(angle<-pi);
angle(i)=angle(i)+2*pi;
end

function [X,Y] = plot_DDMR(Q,AX)
% PLOT_UNICYCLE   Function that generates lines for plotting a unicycle.
%
%    PLOT_UNICYCLE(Q,AX) takes in the axis AX = axis(gca) and the unicycle
%    configuration Q and outputs lines (X,Y) for use, for example, as:
%       fill(X,Y,'b')
%    This must be done in the parent script file.

x     = Q(1);
y     = Q(2);
theta = Q(3);

l1 = 0.02*max([AX(2)-AX(1),AX(4)-AX(3)]);
X = [x,x+l1*cos(theta-2*pi/3),x+l1*cos(theta),x+l1*cos(theta+2*pi/3),x];
Y = [y,y+l1*sin(theta-2*pi/3),y+l1*sin(theta),y+l1*sin(theta+2*pi/3),y];
end
