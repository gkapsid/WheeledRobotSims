-- line follow, detect and bypass obstacle
function sysCall_init()
    -- do some initialization here
     leftWheel = sim.getObjectHandle("DynamicLeftJoint")
    rightWheel = sim.getObjectHandle("DynamicRightJoint")
    leftVS = sim.getObjectHandle("LeftSensor")
    middleVS = sim.getObjectHandle("MiddleSensor")
    rightVS = sim.getObjectHandle("RightSensor")
    uss = sim.getObjectHandle("UltraSonic")
   fl = sim.getObjectHandle("LeftFront")
    rl = sim.getObjectHandle("LeftBack")
    fr = sim.getObjectHandle("RightFront")
    rr = sim.getObjectHandle("RightBack")
  
  -- variable initialization
   lineColour=0
   omega = 0 --robot body angular velocity
   vR =0 -- right wheel velocity
   VL = 0 -- left wheel linear velocity
   v= 0 --robot linear velocity
   distSonar=0.5 -- proximity sensor measurement
    state = 1
    phi =0
    lastTime = 0
    wallFront = false
    wallSide = false
     --minimum turning left time (sec)
   minTurnTime = 0

   
   -- constants
   grey = 0.5
   -- gain for robot's angular velocity (body angular velocity)
   k = 1.5 -- empirically set for line follower 
   kWall = 2 -- empirically set seperation gain for wall following controller
   --linear velocity
   vnom = 0.2
   -- correction gain
   b = 0.2
   -- wheel radius
   wR = 0.027
   --minimum sonar detection distance
    min_dist = 0.15
    --maximum sonar detection distance
   max_dist = 1
     --distance between two rays on the same side (eg two left rays) 3 cm
   a = 0.03
    -- distance of "third" wheel from "back" wheels
   e = 0.075
   -- half distance between wheels
   beta = 0.05
   -- state variable initialization
   state = 1
   -- wanted distance from the obstacle
   dwallSide = 0.15
  --turn linear speed precomputeTurn()
   vturn = 0.2
end

--helper functions
function getDistance(sensor, max_dist)
    local detected, distance
    detected,distance = sim.readProximitySensor(sensor)
    if (detected<1) then
        distance=max_dist
    end
    return distance
end


--calculate phi from each side
function phiside(dist1, dist2) 
    local phiside
    phiside = math.atan((dist1-dist2)/a)
    return phiside
end

--calculate mean distance of each side
--distance from front and rear sensor of the same side
function dmean(dist1, dist2)
    local d
    d = (dist1 + dist2)/2
    return d
end

function getLineColour() 
    local val=0
    value = sim.getVisionSensorImage(middleVS+sim.handleflag_greyscale)
    for i=1, 16 do
        val=val+value[i]
    end
    return val/16
end

function precomputeTurn()
    local wLeft, wRight, t
    wLeft = -vturn/wR
    wRight = vturn /wR
    t = beta*math.pi/2/vturn
    return wLeft, wRight, t
end
-- end of helper functions

function lineFollow()
local wRight, wLeft
--modify linear speed according the ultrasonic distance measurement

        v = vnom*distSonar/max_dist
         -- calculate robot body angular velocity
        omega=-k*(lineColour-grey)
   -- calculate wheels linear velocity
        vL = v-b*omega
        vR = v+b*omega
  
  --angular velocity left
        wLeft=vL/wR
    
    --angular velocity right
        wRight = vR/wR
   
  return wRight, wLeft
  end


function turnLeft()
    local timeElapsed, wLeft, wRight
    wLeft, wRight, minTurnTime = precomputeTurn()
   
   return wRight, wLeft, minTurnTime
end

function followWall()

local mrd, wRight, wLeft --mean distance of right sensors
mdr = dmean(distfr, distrr)
phi = math.atan((distfr-distrr)/a)
d = dwallSide-mdr
gamma = kWall*d
alpha = phi+gamma
--calculate final linear velocity 
  
  --left wheel
   vl = v*(math.cos(alpha) + beta/e*math.sin(alpha))
   --right wheel
   vr = v*(math.cos(alpha) - beta/e*math.sin(alpha))
  
   --angular velocity left
   wLeft=vl/wR
    
    --angular velocity right
   wRight = vr/wR
  
  return wRight, wLeft
end


function sysCall_actuation()
  
    -- set linear speed v
local omegaRight, omegaLeft
  if (state == 1) then
   -- lastTime = sim.getSimulationTime()
    omegaRight, omegaLeft = lineFollow()
elseif (state==2) then -- turn procedure
    omegaRight, omegaLeft, minTurnTime= turnLeft()
  
elseif (state==3) then
 omegaRight, omegaLeft = followWall()
else
 omegaRight = 0
 omegaLeft = 0
end
--print(state)

   
   -- set wheels angular velocity
   sim.setJointTargetVelocity(rightWheel, omegaRight)
   sim.setJointTargetVelocity(leftWheel, omegaLeft )
   
    
end

function sysCall_sensing()
    -- put your sensing code here
    lineColour = getLineColour()
    distSonar = getDistance(uss, max_dist)
    distrl = getDistance(rl, max_dist)
    distfl = getDistance(fl, max_dist)
    distrr = getDistance(rr, max_dist)
    distfr = getDistance(fr, max_dist)
    if (distSonar <0.17) then
        wallFront = true
    end
    if ((distrr<max_dist) and (distfr <max_dist)) then
        wallSide = true
    end
if (state==1) then
    lastTime = sim.getSimulationTime()
    if wallFront then
        state = 2
    end
elseif (state ==2) then

    local timeElapsed = ((sim.getSimulationTime()-lastTime) >minTurnTime)
   
    if wallSide and timeElapsed then 
        state = 3
    end
elseif (state==3) then
    if (lineColour < 0.5) then
    wallFront = false
    wallSide = false
    print('from 3 to state 1')
        state = 1
    end
end
print(state)
end

function sysCall_cleanup()
    -- do some clean-up here
end
   
