function sysCall_init()
    -- do some initialization here
    left_wheel=sim.getObjectHandle('left_joint')
    right_wheel=sim.getObjectHandle('right_joint')
    us=sim.getObjectHandle('Proximity_sensor')
    ir_FL=sim.getObjectHandle('ir_front_left')
    ir_FR=sim.getObjectHandle('ir_front_right')
    ir_RL=sim.getObjectHandle('ir_rear_left')
    ir_RR=sim.getObjectHandle('ir_rear_right')
    init=sim.getObjectHandle('init')
    goal=sim.getObjectHandle('goal')
    robot=sim.getObjectHandle('robotPose')
    
    initPos=sim.getObjectPosition(init,-1)
    goalPos=sim.getObjectPosition(goal,-1)
    
    --Constructive parameters
    wheel_radius=0.03 --wheel radius
    b=0.0565 -- wheel base (wheel separation distance)
    sepUS=0.0815 -- distance of US with respect to the robot wheels
    a=0.03 -- Separation distance between front and rear LIDAR sensors
    sepLIDAR=0.041 -- Lateral separation distance of LIDAR sensors w.r.t. the robot 
    
   -- vref=0.1 -- velocity when following the wall o moving forward
   -- vTurn=0.1 -- velocity when turning
    vref = 0.30
    vTurn = 0.05
    
    objectDist=0.18 -- distance to stop the robot when detecting an object
    
    min_distUS=objectDist-sepUS -- Distance to stop when there's a wall in front of the robot
    kWall=1.8 -- Wall following gain
    e=0.075 -- Distance of an off-center point to perform the kinematic control for wall following 
    wallDist=objectDist-sepLIDAR -- The expected distance to detect a wall with LIDAR sensors
    
    goalDetectedTol=0.1 --Tolerance to point to the goal
    wallDetectedTol=0.03 -- Tolerance to determine that there's a wall when rotating
    goalReachedTol=0.1  --Tolerance to determine the robot has reached the goal
    
    
    lastTime=sim.getSimulationTime()  -- Variable use to measured the time elapsed since the 'last time'
    States={"Point to goal","Move forward","Turn","Follow wall line not crossed","Follow wall","Stop"} -- Names for each defined state of the algorithm
    state=1  -- Initial state
    initRandom()
    direction=1 -- Follow right 1, follow left -1

    max_distUS=0.5 -- Maximum distance returned by the US sensor
    dUS=max_distUS  -- Initial US distance
    max_distLIDAR=1.2 -- Maximum distance returned by the LIDAR sensors
    dFR=max_distLIDAR -- Initial LIDAR distance
    dRR=max_distLIDAR -- Initial LIDAR distance
    dFL=max_distLIDAR -- Initial LIDAR distance
    dRL=max_distLIDAR -- Initial LIDAR distance
    --tolerance = 0.01
    dWallSide = 0.15
    turnTime = 0.02
    
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
    elseif (state==3) then
        wTurnL,wTurnR,turnTime=precomputeTurn(vTurn,direction,wheel_radius,b)
        wL=wTurnL
        wR=wTurnR
    elseif (state==4) then
        wL,wR=followWall(vref,direction,dFL,dRL,dFR,dRR,wallDist)
    elseif (state==5) then
        wL,wR=followWall(vref,direction,dFL,dRL,dFR,dRR,wallDist)
    elseif (state==6) then
        wL=0
        wR=0
    end
    sim.setJointTargetVelocity(left_wheel,wL)
    sim.setJointTargetVelocity(right_wheel,wR)
end

function sysCall_sensing()
    -- put your sensing code here
    --TODO: Read proximity sensors
    dFR = getDistance(ir_FR, max_distLIDAR)
    dRR = getDistance(ir_RR, max_distLIDAR)
    dFL = getDistance(ir_FL, max_distLIDAR)
    dRL = getDistance(ir_RL, max_distLIDAR)
  
  --TODO: Update robot position
    pose = getRobotPose()
    
    --State machine
    if (state==1) then
        --TODO: Check if it is pointing to goal
        pointingToGoal = PointingToGoal(pose,initPos,goalPos,goalDetectedTol)
       -- print(pose)
       -- print(pointingToGoal)
        if pointingToGoal then
            state=2
        end
    elseif (state==2) then
        --TODO: Check if it is pointing to goal
          pointingToGoal = PointingToGoal(pose,initPos,goalPos,goalDetectedTol)
      
        --TODO: Check if goal has been reached
        goalReached = GoalReached(pose,goalPos,goalReachedTol)
        
        --TODO: Check if there's an object in front
        dUS = getDistance(us,max_distUS)
        print(dUS)
        if goalReached then
            state=6
        elseif not pointingToGoal then
            state=1
        elseif dUS<min_distUS then
            lastTime=sim.getSimulationTime()
            direction=selectRandomDirection()
            state=3
        end
    elseif (state==3) then
        --TODO: Check wall on the side
        wall = wallDetected(direction,dFL,dRL,dFR,dRR,dWallSide,wallDetectedTol)
        
        --TODO: Check time elapsed
        timeElapsed = (sim.getSimulationTime()-lastTime)>turnTime
        
        if wall and timeElapsed then
            state=4
        end
    elseif (state==4) then
        --TODO: Check side of the line
        lineSide = LineSide(pose,initPos,goalPos)
        if lineSide==direction then
            state=5
        end
    elseif (state==5) then
        --TODO: Check side of the line
        lineSide = LineSide(pose,initPos,goalPos)

        if lineSide~=direction then
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

function wallDetected(direction,dFL,dRL,dFR,dRR,dWallSide,tolerance)
    -- direction: 1 wall on the right, -1 wall on the left
    -- dFL,dRL: distance of LIDAR sensors on the left
    -- dFR,dRR: distance of LIDAR sensors on the right
    -- dWallSide: expected separation distance of the wall
    -- tolerance: admissible distance to consider that there's a wall
    -- Returns true or false
    local dF,dR,front,rear
    if (direction==1) then
        dF=dFR
        dR=dRR
    else
        dF=dFL
        dR=dRL
    end
    front=math.abs(dF-dWallSide)<tolerance
    rear=math.abs(dR-dWallSide)<tolerance
    return front and rear
end

function followWall(v,direction,dFL,dRL,dFR,dRR,dWallSide)
    -- dFL,dRL: distance of LIDAR sensors on the left
    -- dFR,dRR: distance of LIDAR sensors on the right
    -- dWallSide: expected separation distance of the wall
    -- wheel_radius,a,b: Constructuve parameters
    -- v,kWall,e: Parameters for wall following algorithm
    -- Returns wheels velocities
    local phi,d,alpha,wL,wR
    if (direction==1) then  -- follow right wall
        phi=math.atan((dFR-dRR)/a)
        d=(0.5*(dFR+dRR)-dWallSide)
    else -- follow left wall
        phi=math.atan((dRL-dFL)/a)
        d=(dWallSide-0.5*(dFL+dRL))
   --   d = (0.5*(dFL+dRL) - dWallSide)
    end
    gamma=kWall*d
    alpha=phi+gamma
    wL=(v/wheel_radius)*(math.cos(alpha)+(b/e)*math.sin(alpha))
    wR=(v/wheel_radius)*(math.cos(alpha)-(b/e)*math.sin(alpha))
    return wL,wR
end

function LineSide(pose,initPos,goalPos)
    -- pose: list with x,y,theta values of the robot
    -- initPose: list with x,y of the initial position
    -- goalPose: list with x,y of the goal position
    -- Returns 1 on the left side and -1 on the right side
    local d,dGoalIni,xDiff1,yDiff1,xDiff2,yDiff2
    xDiff1=initPos[1]-goalPos[1]
    yDiff1=goalPos[2]-initPos[2]
    xDiff2=initPos[1]-pose[1]
    yDiff2=initPos[2]-pose[2]
    dGoalIni=math.sqrt(xDiff1^2+yDiff1^2)
    d=(xDiff2*yDiff1+yDiff2*xDiff1)/dGoalIni
    if d>0 then
        return 1
    else
        return -1
    end
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


function getDistance(sensor,max_dist)
    local detected, distance
    detected,distance=sim.readProximitySensor(sensor)
    if (detected<1) then
        distance=max_dist
    end
    return distance
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
