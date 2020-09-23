function sysCall_init()
    -- do some initialization here
     leftWheel = sim.getObjectHandle("DynamicLeftJoint")
    rightWheel = sim.getObjectHandle("DynamicRightJoint")
    leftVS = sim.getObjectHandle("LeftSensor")
    middleVS = sim.getObjectHandle("MiddleSensor")
    rightVS = sim.getObjectHandle("RightSensor")
   -- sim.setJointTargetVelocity(leftWheel, 2)
   -- sim.setJointTargetVelocity(rightWheel, 2)
 -- last_time=sim.getSimulationTime()
  -- ccw = false
  
  -- variable initialization
   lineColour=0
   omega = 0 --robot body angular velocity
   vR =0 -- right wheel velocity
   VL = 0 -- left wheel linear velocity
   v= 0 --robot linear velocity
   
   -- constants
   grey = 0.5
   -- gain for robot's angular velocity (body angular velocity)
   k = 2 -- empirically set
   --linear velocity
   v = 0.1
   -- correction gain
   b = 0.2
   -- wheel radius
   wR = 0.027
   
end

function sysCall_actuation()
    -- put your actuation code here
   -- print(lineColour)
    -- set linear speed v

--modify linear speed according the ultrasonic distance measurement
  -- if (distSonar  > min_dist) then 
  --      v = vnom*distSonar/max_dist
  --  else
  --      v=0
  --  end
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
end

function sysCall_cleanup()
    -- do some clean-up here
end

-- See the user manual or the available code snippets for additional callback functions and details
-- helper function
function getLineColour() 
    local val=0
    value = sim.getVisionSensorImage(middleVS+sim.handleflag_greyscale)
    for i=1, 16 do
        val=val+value[i]
    end
    return val/16
end

   
