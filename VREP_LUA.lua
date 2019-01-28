//This is an example of what the lua code looks like to interface between ROS and MATLAB using a Pioneer 3-DX


function vel_callback(msg)
    sim.addStatusbarMessage('Pioneer 1 received vRight value: '..msg.x)
    sim.addStatusbarMessage('Pioneer 1 received vLeft value:  '..msg.y)

    -- Receive and Set Velocity
    sim.setJointTargetVelocity(motorLeft,msg.y)
    sim.setJointTargetVelocity(motorRight,msg.x)
end

function sysCall_init()
    -- The child script initialization
    motorLeft=sim.getObjectHandle("Pioneer_p3dx_leftMotor")
    motorRight=sim.getObjectHandle("Pioneer_p3dx_rightMotor")
    pioneer=sim.getObjectHandle("Pioneer_p3dx")
   
-- Check if the required RosInterface is there:
    moduleName=0
    index=0
    rosInterfacePresent=false
    while moduleName do
        moduleName=sim.getModuleName(index)
        if (moduleName=='RosInterface') then
            rosInterfacePresent=true
        end
        index=index+1
    end

    -- Prepare the vector3 publisher and subscriber:
    if rosInterfacePresent then
        pubPose=simROS.advertise('/pioneerPose_1', 'geometry_msgs/Vector3')
        subVel=simROS.subscribe('/desiredVel_1','geometry_msgs/Vector3','vel_callback')
    end
end

function sysCall_actuation()
    -- Send an updated simulation time message, and send the transform of the object attached to this script:
    if rosInterfacePresent then
        -- sim.getObjectPosition returns a table of three values x, y, z
        posePioneer = sim.getObjectPosition(pioneer, -1)
        orientPioneer = sim.getObjectOrientation(pioneer, -1)
        simROS.publish(pubPose, {x=posePioneer[1],y=posePioneer[2],z=orientPioneer[3]})
        
        -- The following has been moved up into the callback:
        -- sim.setJointTargetVelocity(leftMotor, vLeft)
    end
end

function sysCall_cleanup()
    -- Following not really needed in a simulation script (i.e. automatically shut down at simulation end):
    if rosInterfacePresent then
        simROS.shutdownPublisher(pubPose)
        simROS.shutdownSubscriber(subVel)
    end
end


