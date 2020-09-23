function sysCall_init()
    -- do some initialization here
    leftWheel = sim.getObjectHandle("DynamicLeftJoint")
    rightWheel = sim.getObjectHandle("DynamicRightJoint")
    uss = sim.getObjectHandle("UltraSonic")
    fl = sim.getObjectHandle("LeftFront")
    rl = sim.getObjectHandle("LeftBack")
    fr = sim.getObjectHandle("RightFront")
    rr = sim.getObjectHandle("RightBack")
    leftSensor = sim.getObjectHandle("LeftSensor")
    middleSensor = sim.getObjectHandle("MiddleSensor")
    rightSensor = sim.getObjectHandle("RightSensor")
    distSonar=0
    distrl=0
    distfl =0
    distfr = 0
    distrr = 0
    phiR = 0
    phiL = 0
    phi =0
    
    --CONSTANTS
    -- maximum and minimum detection distance for ultrasonic sensor
    max_dist = 1
    min_dist = 0.15
    --distance between two rays on the same side (eg two left rays) 3 cm
   a = 0.03
   -- factor chosen arbitrarily, I don't know exactly its role
   k = 1
   -- maximum expected linear speed 
   vnom = 1
   -- half distance between wheels
   beta = 0.05
   -- wheel radius
   wR = 0.027
   -- distance of "third" wheel from "back" wheels
   e = 0.075

end

--helper functions

--get sensor distance in m
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
-- end of helper functions

function sysCall_actuation()
   -- Follow wall controller
   -- at least four sensors necessary (two at each side)
   --calculate mean distance and angle from left wall
   dl = dmean(distrl, distfl)
   phiL = phiside(distrl, distfl)
   
   --calculate mean distance and angle from right wall
   phiR = phiside(distfr, distrr)
   dr = dmean(distrr, distfr)
  
   --caclulate heading error
   phi = (phiL + phiR)/2
  
   -- calculate separation error (k arbtitrary coefficient initialized previously)
   gama = k*(dr-dl)
   
   --"third wheel" angle alpha
   alpha = phi + gama
   
   -- set linear speed v

--modify linear speed according the ultrasonic distance measurement
   if (distSonar  > min_dist) then 
        v = vnom*distSonar/max_dist
    else
        v=0
    end
    
   --calculate final linear velocity 
  
  --left wheel
   vl = v*(math.cos(alpha) + beta/e*math.sin(alpha))
   --right wheel
   vr = v*(math.cos(alpha) - beta/e*math.sin(alpha))
  
   --angular velocity left
   omegaLeft=vl/wR
    
    --angular velocity right
   omegaRight = vr/wR
   
   --set calculated angular velocities
   sim.setJointTargetVelocity(rightWheel, omegaRight)
   sim.setJointTargetVelocity(leftWheel, omegaLeft )
  
end

function sysCall_sensing()
    -- put your sensing code here
    distSonar = getDistance(uss, max_dist)
    distrl = getDistance(rl, max_dist)
    distfl = getDistance(fl, max_dist)
    distrr = getDistance(rr, max_dist)
    distfr = getDistance(fr, max_dist)
end

function sysCall_cleanup()
    -- do some clean-up here
end

-- See the user manual or the available code snippets for additional callback functions and details
