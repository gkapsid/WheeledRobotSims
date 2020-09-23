function sysCall_init()
    -- do some initialization here
     leftWheel = sim.getObjectHandle("DynamicLeftJoint")
    rightWheel = sim.getObjectHandle("DynamicRightJoint")
    leftVS = sim.getObjectHandle("LeftSensor")
    middleVS = sim.getObjectHandle("MiddleSensor")
    rightVS = sim.getObjectHandle("RightSensor")
    uss = sim.getObjectHandle("UltraSonic")
  
  
  -- variable initialization
   lineColour=0
   omega = 0 --robot body angular velocity
   vR =0 -- right wheel velocity
   VL = 0 -- left wheel linear velocity
   v= 0 --robot linear velocity
    distSonar=0 -- proximity sensor measurement

   
   -- constants
   grey = 0.5
   -- gain for robot's angular velocity (body angular velocity)
   k = 1.5 -- empirically set
   --linear velocity
   vnom = 0.1
   -- correction gain
   b = 0.2
   -- wheel radius
   wR = 0.027
   --minimum sonar detection distance
    min_dist = 0.15
    --maximum sonar detection distance
   max_dist = 1
   
end

function getDistance(sensor, max_dist)
    local detected, distance
    detected,distance = sim.readProximitySensor(sensor)
    if (detected<1) then
        distance=max_dist
    end
    return distance
end

function sysCall_actuation()
    -- put your actuation code here
 
--modify linear speed according the ultrasonic distance measurement
  if (distSonar  > min_dist) then 
        v = vnom*distSonar/max_dist
    else
        v=0
  end
   -- calculate robot body angular velocity
   omega=-k*(lineColour-grey)
   -- calculate wheels linear velocity
   vL = v-b*omega
   vR = v+b*omega
  
  --angular velocity left
   omegaLeft=vL/wR
    
    --angular velocity right
   omegaRight = vR/wR
   
   -- set wheels angular velocity
    sim.setJointTargetVelocity(rightWheel, omegaRight)
   sim.setJointTargetVelocity(leftWheel, omegaLeft )
   
    
end

function sysCall_sensing()
    -- put your sensing code here
    lineColour = getLineColour()
    distSonar = getDistance(uss, max_dist)

end

function sysCall_cleanup()
    -- do some clean-up here
end

-- helper function
function getLineColour() 
    local val=0
    value = sim.getVisionSensorImage(middleVS+sim.handleflag_greyscale)
    for i=1, 16 do
        val=val+value[i]
    end
    return val/16
end

   
