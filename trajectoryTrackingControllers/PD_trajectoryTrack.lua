function sysCall_init()
    -- do some initialization here
    left_wheel=sim.getObjectHandle('Pioneer_p3dx_leftMotor')
    right_wheel=sim.getObjectHandle('Pioneer_p3dx_rightMotor')
   
  --see customization code in ref_point
  
  ref_point = sim.getObjectHandle('ref_point')
    robot_pose=sim.getObjectHandle('robot_pose')
     
    --constants and design parameter choice
    
    -- wheel separation distance
    b = 0.1655
    --wheel radius
    R = 0.1955/2
    --off center point (chosen arbitrarily --> higher value, smoother response, less acurate)
    e = 0.08 
    --Kp proportional gain (chosen arbitrarily) that is multiplied with error between reference off center point
    --and robot off center point (ref_point, robot off center point ??)
    Kp = 2
     -- Kd velocity error gain (differential gain) arbitrarily chosen ...
Kd = 2     
end

function sysCall_actuation()

    --PD kinecmatic control for trajectory tracking
   
local robotPose,pose_ref, velocity_ref,invJacobi, robotVel
robotPose, robotVel = calculateHitta()
pose_ref, velocity_ref = calculateHittaRef()
invJacobi = jacobian()
print(robotPose) 

interVec = {{Kd*(velocity_ref[1]-robotVel[1])+Kp*(pose_ref[1]-robotPose[1])},{Kd*(velocity_ref[2]-robotVel[2])+Kp*(pose_ref[2]-robotPose[2])}}
omegaWheel = {(invJacobi[1][1]*interVec[1][1]+invJacobi[1][2]*interVec[2][1]), (invJacobi[2][1]*interVec[1][1]+invJacobi[2][2]*interVec[2][1]) }
    sim.setJointTargetVelocity(left_wheel,omegaWheel[1])
    sim.setJointTargetVelocity(right_wheel,omegaWheel[2])

end

function sysCall_sensing()
    -- put your sensing code here
    pose=updateRobotPose()
    
    
end

function sysCall_cleanup()
    -- do some clean-up here
end

function getTrajectoryPoint()
    local position, orientation
    local linear_vel, angular_vel
    local ptraj, vtraj
    position = sim.getObjectPosition(ref_point, -1)
    orientation=sim.getObjectOrientation(ref_point,-1)
    linear_vel, angular_vel =sim.getObjectVelocity(ref_point)
    if (orientation[3] >0) then
        ptraj = {position[1], position[2], orientation[2]-math.pi/2}
    else
        ptraj={position[1], position[2], math.pi/2-orientation[2]}
    end
    
    
    vtraj = {linear_vel[1],linear_vel[2],angular_vel[3]} 
    return ptraj, vtraj
end

function updateRobotPose() --pose and velocity for PD control
    local pose, linear_vel, angular_vel
    local vrobot
    position = sim.getObjectPosition(robot_pose, -1)
    orientation = sim.getObjectOrientation(robot_pose,-1)
    pose={position[1],position[2],orientation[3]}
    linear_vel, angular_vel = sim.getObjectVelocity(robot_pose)
    vrobot = {linear_vel[1],linear_vel[2],angular_vel[3]}
    return pose, vrobot
end

function jacobian()
local robotPose = updateRobotPose()
local theta = robotPose[3]
local jacobi
jacobi = {}
for i = 1, 2 do
    jacobi[i] = {}

    for j = 1, 2 do
        jacobi[i][j] = 0 -- Fill the values here
    end
end
    jacobi[1][1] = (1/R)*(math.cos(theta)+(b/e)*math.sin(theta))
    jacobi[2][1] = (1/R)*(math.cos(theta)-(b/e)*math.sin(theta))
    jacobi[1][2] = (1/R)*(math.sin(theta)-(b/e)*math.cos(theta))
    jacobi[2][2] = (1/R)*(math.sin(theta)+(b/e)*math.cos(theta))

return jacobi
end

function calculateHitta()
--robot point position hitta 
local robotPose, robotVel= updateRobotPose()
local x = robotPose[1]
local y = robotPose[2]
local theta = robotPose[3]
local Hitta
Hitta={x+e*math.cos(theta) , y+e*math.sin(theta)}

--robot pose velocity hittaDot
local xdot = robotVel[1]
local ydot = robotVel[2]
local thetadot = robotVel[3]
local HittaDot = {xdot-e*thetadot*math.sin(theta), ydot+e*thetadot*math.cos(theta)}

return Hitta,HittaDot
end

function calculateHittaRef()
-- reference trajectory point position
local pathPose, pathVelocity = getTrajectoryPoint()
local xref = pathPose[1]
local yref = pathPose[2]
local thetaRef = pathPose[3]
local HittaRef
HittaRef={xref+e*math.cos(thetaRef) , yref+e*math.sin(thetaRef)}

--reference trajectory point velocity
local xrefdot = pathVelocity[1]
local yrefdot = pathVelocity[2]
local thetarefdot = pathVelocity[3]
local HittaRefdot = {xrefdot-e*thetarefdot*math.sin(thetaRef), yrefdot+e*thetarefdot*math.cos(thetaRef)}
return HittaRef, HittaRefdot 
end
