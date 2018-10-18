%Project
clear all
close all
clc

qd0 = [2.5,1.5,pi/2]; %leader initial pose
q0 = [2,1,pi/2];
T = 0.05;
tf = 40;
R = 1;
xc = 1.5;
yc = 1.5;
alpha = 0.5;
doff = 0.6;
Kv = 5;
Ki = 0.5; %0.5
Kd = 0.8; %Not used
Kgamma = 1;
l = 0.381;
r = 0.0925; %unused
lf = 0.16; %Follower axle width
rf = 0.06; %Follower wheel radius
t = 0;
leaderpose = qd0;
followerpose = q0;
integral_old = 0;
previous_error = 0;
figure(1)
    hold on
    
[vrep, clientID, rtn, followerrobot,leaderrobot]= init_VREP();

pause(T);
leaderpose = pose_VREP(vrep,clientID,leaderrobot);
followerpose = pose_VREP(vrep,clientID,followerrobot);

while t < tf  % main timing loop
   leaderpose = pose_VREP(vrep,clientID,leaderrobot);
   leaderv = leadervelocity(Kgamma,leaderpose,t);
   move_VREP(l,leaderv,vrep,clientID,leaderrobot);
   leaderpose = pose_VREP(vrep,clientID,leaderrobot);
   [followerv,integral_old,previous_error] = followervelocity(leaderpose,integral_old,previous_error,followerpose,doff,T,Kgamma,Kv,Ki,Kd,lf);
   move_VREP(lf,followerv,vrep,clientID,followerrobot);
   followerpose = pose_VREP(vrep,clientID,followerrobot);
   followerpose(3) = followerpose(3) - 90;
   figure(1)
        plot(leaderpose(1),leaderpose(2),'b.');
        plot(followerpose(1),followerpose(2),'r.');
    pause(T);
    t = t + T;
end
velocity = [0,0];
move_VREP(l,velocity,vrep,clientID,leaderrobot);
move_VREP(lf,velocity,vrep,clientID,followerrobot);
end_VREP(vrep,clientID,leaderrobot);
end_VREP(vrep,clientID,followerrobot);

function vel = leadervelocity(Kgamma,p_current,t)
    


    k1 = 0.2;
    k2 = 0.2;
    a = 2;
    
    %x = k1 * p_current(1);
    %y = a *sin(k2 * p_current(1));
    
     xd_dot = k1;
     yd_dot = a* k2 *cos(k2*t);
     
     desired_theta=atan2(yd_dot,xd_dot);
     desired_v = sqrt((xd_dot^2)+(yd_dot^2));
    
    vel(1) = desired_v;
    vel(2) = (desired_theta - p_current(3))* Kgamma;

end

function [vel,integral_new,error] = followervelocity(leaderpose,integral_old,previous_error,followerpose,doff,T,Kgamma,Kv,Ki,Kd,l)
    
    error = sqrt(((leaderpose(1) - followerpose(1))^2)+((leaderpose(2) - followerpose(2))^2))- doff;
    
    integral_new = integral_old + (error * T);
    
    derivative = (error - previous_error) / T;
    
    vel(1) = (Kv*error)+(Ki*integral_new)+(Kd*derivative);
    
    ThetaPrime = atan2((leaderpose(2)-followerpose(2)),(leaderpose(1)-followerpose(1)));
    ThetaTilde = ThetaPrime - followerpose(3);
    ThetaTilde = mod(ThetaTilde,2*pi);
    
    if ThetaTilde > pi
        ThetaTilde = ThetaTilde - 2*pi;
    end
    if ThetaTilde < -pi
        ThetaTilde = ThetaTilde + 2*pi;
    end
    gamma = Kgamma*ThetaTilde;
    
    gamma = sign(gamma)*min(80*pi/180,abs(gamma));
    vel(2) = (vel(1)/l)*tan(gamma);
end

%--------------------------------------------------------------------------
%Functions Using Vrep
%--------------------------------------------------------------------------
function  [vrep, clientID, rtn, followerrobot,leaderrobot]= init_VREP()

vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID =vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

[rtn,followerrobot(1,1)] = vrep.simxGetObjectHandle(clientID,'BBBTest', vrep.simx_opmode_oneshot_wait);
[rtn,followerrobot(1,2)] = vrep.simxGetObjectHandle(clientID,'RightMotor', vrep.simx_opmode_oneshot_wait);
[rtn,followerrobot(1,3)] = vrep.simxGetObjectHandle(clientID,'LeftMotor', vrep.simx_opmode_oneshot_wait);
[rtn,leaderrobot(1,1)] = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx', vrep.simx_opmode_oneshot_wait);
[rtn,leaderrobot(1,2)] = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor', vrep.simx_opmode_oneshot_wait);
[rtn,leaderrobot(1,3)] = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor', vrep.simx_opmode_oneshot_wait);

vrep.simxGetObjectPosition(clientID,followerrobot(1,1),-1,vrep.simx_opmode_streaming);
vrep.simxGetObjectOrientation(clientID,followerrobot(1,1),-1,vrep.simx_opmode_streaming);
vrep.simxGetObjectPosition(clientID,leaderrobot(1,1),-1,vrep.simx_opmode_streaming);
vrep.simxGetObjectOrientation(clientID,leaderrobot(1,1),-1,vrep.simx_opmode_streaming);
end


function move_VREP(l,velocity,vrep,clientID,robot)
    nuR = (2*velocity(1)+l*velocity(2))/2; % [m/s] robot's right wheel's linear speed
    nuL = (2*velocity(1)-l*velocity(2))/2; % [m/s] robot's left wheel's linear speed

    vrep.simxPauseCommunication(clientID,1);
    vrep.simxSetJointTargetVelocity(clientID,robot(1,2),nuR,vrep.simx_opmode_oneshot); 
    vrep.simxSetJointTargetVelocity(clientID,robot(1,3),nuL,vrep.simx_opmode_oneshot); 
    vrep.simxPauseCommunication(clientID,0);
end

function p_current= pose_VREP(vrep,clientID,robot)

    [~,tmp_pos] = vrep.simxGetObjectPosition(clientID,robot(1,1),-1,vrep.simx_opmode_buffer);
    [~,tmp_angle] = vrep.simxGetObjectOrientation(clientID,robot(1,1),-1,vrep.simx_opmode_buffer);
    p_current(1:2) = tmp_pos(1:2); %temp varibles to maintain index
    p_current(3) = tmp_angle(3); %possibly not needed

end

function end_VREP(vrep,clientID,robot)

vrep.simxPauseCommunication(clientID,1);
vrep.simxSetJointTargetVelocity(clientID,robot(1,2),0,vrep.simx_opmode_oneshot); 
vrep.simxSetJointTargetVelocity(clientID,robot(1,3),0,vrep.simx_opmode_oneshot); 
vrep.simxPauseCommunication(clientID,0);

vrep.simxGetPingTime(clientID);
% Now close the connection to V-REP:	
vrep.simxFinish(clientID);    
vrep.delete(); % call the destructor!

end