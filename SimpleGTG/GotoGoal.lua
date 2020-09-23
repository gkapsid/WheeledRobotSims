function sysCall_init()
    -- do some initialization here
    left_wheel=sim.getObjectHandle('left_joint')
    right_wheel=sim.getObjectHandle('right_joint')
 
    init=sim.getObjectHandle('init')
    goal=sim.getObjectHandle('goal')
    robot=sim.getObjectHandle('robotPose')
    
    initPos=sim.getObjectPosition(init,-1)
    goalPos=sim.getObjectPosition(goal,-1)
    
    --Constructive parameters
    wheel_radius=0.03 --wheel radius
    b=0.0565 -- wheel base (wheel separation distance)
    a=0.03 -- Separation distance between front and rear LIDAR sensors
 
   vTurn=0.04 -- velocity when turning
    vref = 0.40
   -- vTurn = 0.05
    
    objectDist=0.18 -- distance to stop the robot when detecting an object
    

    e=0.075 -- Distance of an off-center point to perform the kinematic control for wall following 
 
    goalDetectedTol=0.2 --Tolerance to point to the goal
  goalReachedTol=0.2  --Tolerance to determine the robot has reached the goal
    
    
    lastTime=sim.getSimulationTime()  -- Variable use to measured the time elapsed since the 'last time'
    States={"Point to goal","Move forward","Turn","Follow wall line not crossed","Follow wall","Stop"} -- Names for each defined state of the algorithm
    state=1  -- Initial state
    initRandom()
    direction=1 -- Follow right 1, follow left -1

    turnTime = 0.01
    
end

function sysCall_actuation()
    -- put your actuation code here
    local wL,wR,v
    if (state==1) then
        wTurnL,wTurnR,turnTime=precomputeTurn(vTurn,direction,wheel_radius,b)
        wL=wTurnL
        wR=wTurnR
        
      -- print(wL, wR)
    elseif (state==2) then
        wL=vref/wheel_radius
        wR=vref/wheel_radius
 
    elseif (state==6) then
        wL=0
        wR=0
    end
    sim.setJointTargetVelocity(left_wheel,wL)
    sim.setJointTargetVelocity(right_wheel,wR)
end

function sysCall_sensing()
    -- put your sensing code here
  
  --Update robot position
    pose = getRobotPose()
    
    --State machine
    if (state==1) then
        --Check if it is pointing to goal
        pointingToGoal = PointingToGoal(pose,initPos,goalPos,goalDetectedTol)
     
        if pointingToGoal then
            state=2
        end
    elseif (state==2) then
        --Check if it is pointing to goal
          pointingToGoal = PointingToGoal(pose,initPos,goalPos,goalDetectedTol)
      
        --Check if goal has been reached
        goalReached = GoalReached(pose,goalPos,goalReachedTol)
       
        if goalReached then
            state=6
        elseif not pointingToGoal then
            state=1
    
        end
   
    end
    print(state)
end

function sysCall_cleanup()
    -- do some clean-up here
end

function getRobotPose()
    -- Returns the robot pose (list with x,y and theta)
    local pose
    position=sim.getObjectPosition(robot,-1)
    orientation=sim.getObjectOrientation(robot,-1)
    pose={position[1],position[2],orientation[3]}
    return pose
end

function PointingToGoal(pose,initPos,goalPos,tolerance)
    -- pose: list with x,y,theta values of the robot
    -- initPose: list with x,y of the initial position
    -- goalPose: list with x,y of the goal position
    -- tolerance: admissible angle (in radians)
    -- Returns true or false and the turning direction (1 or -1)
    local angle_diff,pointing,direction
    angle_diff=math.atan2(goalPos[2]-initPos[2],goalPos[1]-initPos[1])-pose[3]
    if (math.abs(angle_diff)<tolerance) then
        pointing=true
    else
        pointing=false
    end
    if (angle_diff>0) then
        direction=1
    else
        direction=-1
    end
    return pointing,direction
end

function precomputeTurn(vturn,direction,wheel_radius,b)
    -- vturn: turning speed
    -- direction: 1-> Follow right, 1-> Follow left
    -- wheel_radius,b: constructive parameters
    -- Returns wheels velocities and turning time
    local wL,wR,t
    wL=-direction*vturn/wheel_radius
    wR=direction*vturn/wheel_radius
    t=b*(math.pi/2)/vturn
    return wL,wR,t
end


function GoalReached(pose,goalPos,tolerance)
    -- pose: list with x,y,theta values of the robot
    -- goalPose: list with x,y of the goal position
    -- tolerance: admissible distance to consider that the goal has been reached
    -- Returns true or false
    local dGoal
    dGoal=math.sqrt((goalPos[1]-pose[1])^2+(goalPos[2]-pose[2])^2)
    return dGoal<tolerance
end

function initRandom()
    math.randomseed(os.time())
    for i=1,5,1 do
        math.random()
    end
end

function selectRandomDirection()
    local n,direction
    n=math.random(0,1)
    if (n<0.5) then
        direction=1
    else
        direction=-1
    end
    return direction
end
