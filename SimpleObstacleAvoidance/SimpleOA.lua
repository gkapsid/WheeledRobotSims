setLeds=function(elHandle,left,middle,right)
    simSetUIButtonProperty(elHandle,8,sim.buttonproperty_staydown)
    simSetUIButtonProperty(elHandle,16,sim.buttonproperty_staydown)
    simSetUIButtonProperty(elHandle,24,sim.buttonproperty_staydown)
    if (left) then
        simSetUIButtonProperty(elHandle,8,sim.buttonproperty_staydown+sim.buttonproperty_isdown)
    end
    if (middle) then
        simSetUIButtonProperty(elHandle,16,sim.buttonproperty_staydown+sim.buttonproperty_isdown)
    end
    if (right) then
        simSetUIButtonProperty(elHandle,24,sim.buttonproperty_staydown+sim.buttonproperty_isdown)
    end
end

function sysCall_threadmain()
    -- Put some initialization code here:
    display=simGetUIHandle("sensorDisplay")
    setLeds(display,false,false,false)
    objHandle=sim.getObjectAssociatedWithScript(sim.handle_self)
    result,robotName=sim.getObjectName(objHandle)
    simSetUIButtonLabel(display,0,robotName)
    lineTracerBase=sim.getObjectHandle("LineTracerBase")
    leftSensor=sim.getObjectHandle("LeftSensor")
   -- middleSensor=sim.getObjectHandle("MiddleSensor")
    rightSensor=sim.getObjectHandle("RightSensor")
    leftJoint=sim.getObjectHandle("LeftJoint")
    rightJoint=sim.getObjectHandle("RightJoint")
    leftJointDynamic=sim.getObjectHandle("DynamicLeftJoint")
    rightJointDynamic=sim.getObjectHandle("DynamicRightJoint")
    middleProxSens = sim.getObjectHandle("MiddleProxSens")
    rightProxSens = sim.getObjectHandle("RightProxSens")
    leftProxSens = sim.getObjectHandle("LeftProxSens")

    nominalLinearVelocity=0.3
    wheelRadius=0.027
    interWheelDistance=0.119
    initialVehicleZpos=sim.getObjectPosition(objHandle,sim.handle_parent)[3]
    previousSimulationTime=sim.getSimulationTime()

    -- We want next while-loop to be executed exactly once every main script pass, but since
    -- this script runs in a thread, we explicitely switch threads at the end of the while-loop
    -- Next instruction makes sure one full pass of the while-loop can be executed before switching threads:
    sim.setThreadSwitchTiming(99)

    while sim.getSimulationState()~=sim.simulation_advancing_abouttostop do
        simulationTime=sim.getSimulationTime()
        dt=simulationTime-previousSimulationTime
        previousSimulationTime=simulationTime
        state=sim.getSimulationState()

        s=sim.getObjectSizeFactor(objHandle) -- in case we wanna scale the vehicle during simulation

        -- Check the user interface:
        b=simGetUIButtonProperty(display,4)
        dyn=(sim.boolAnd32(b,sim.buttonproperty_isdown)==0)
        if (dynamicSimulation==nil) or (dynamicSimulation~=dyn) then
            dynamicSimulation=dyn
            p=sim.boolOr32(sim.getModelProperty(objHandle),sim.modelproperty_not_dynamic)
            if (dynamicSimulation) then
                sim.setModelProperty(objHandle,p-sim.modelproperty_not_dynamic)
            else
                sim.setModelProperty(objHandle,p)
                -- Make also sure the vahicle is flat on the ground:
                p=sim.getObjectPosition(objHandle,sim.handle_parent)
                o=sim.getObjectOrientation(objHandle,sim.handle_parent)
                p[3]=initialVehicleZpos
                o[1]=0
                o[2]=0
                sim.setObjectPosition(objHandle,sim.handle_parent,p)
                sim.setObjectOrientation(objHandle,sim.handle_parent,o)
            end
        end

        -- Read the sensors:
        sensorReading={false,false,false}
        sensorReading[1]=(sim.readVisionSensor(leftSensor)==1)
       -- sensorReading[2]=(sim.readVisionSensor(middleSensor)==1)
        sensorReading[3]=(sim.readVisionSensor(rightSensor)==1)
        
        proxSensReadingM = sim.readProximitySensor(middleProxSens)
        proxSensReadingL = sim.readProximitySensor(leftProxSens)
        proxSensReadingR = sim.readProximitySensor(rightProxSens)


        -- Set the sensor indicators:
        setLeds(display,sensorReading[1],sensorReading[2],sensorReading[3])

        -- Decide about left and right velocities:
        linearVelocityLeft=nominalLinearVelocity*0.5
        linearVelocityRight=nominalLinearVelocity*0.5
        if (sensorReading[1]==false) then
            linearVelocityLeft=linearVelocityLeft*0.3
        end
        if (sensorReading[3]==false) then
            linearVelocityRight=linearVelocityRight*0.3
        end
        
        if (proxSensReadingM == 1) then 
            linearVelocityLeft = 0
            linearVelocityRight = 0
        end
        if ((proxSensReadingL == 1) ) then
                linearVelocityLeft=nominalLinearVelocity*0.3
                linearVelocityRight=-nominalLinearVelocity*s
                end
        if ((proxSensReadingR == 1) ) then
                linearVelocityLeft=-nominalLinearVelocity*s
                linearVelocityRight=nominalLinearVelocity*0.3
        end
        if ((proxSensReadingL == 0) and (proxSensReadingR == 0) and (proxSensReadingM == 1)) then
            linearVelocityLeft=-nominalLinearVelocity
            linearVelocityRight=nominalLinearVelocity
        end
        
       

        print(proxSensReadingR)

        -- Now make it move!
        if (dynamicSimulation) then
            sim.setJointTargetVelocity(leftJointDynamic,linearVelocityLeft/(s*wheelRadius))
            sim.setJointTargetVelocity(rightJointDynamic,linearVelocityRight/(s*wheelRadius))
        else
            dt=sim.getSimulationTimeStep()
            p=sim.getJointPosition(leftJoint)
            sim.setJointPosition(leftJoint,p+linearVelocityLeft*dt/(s*wheelRadius))
            p=sim.getJointPosition(rightJoint)
            sim.setJointPosition(rightJoint,p+linearVelocityRight*dt/(s*wheelRadius))
            linMov=dt*(linearVelocityLeft+linearVelocityRight)/2.0
            rotMov=dt*math.atan((linearVelocityRight-linearVelocityLeft)/(s*interWheelDistance))


            position=sim.getObjectPosition(lineTracerBase,-1)
            orientation=sim.getObjectOrientation(lineTracerBase,-1)
            xDir={math.cos(orientation[3]),math.sin(orientation[3]),0.0}
            position[1]=position[1]+xDir[1]*linMov
            position[2]=position[2]+xDir[2]*linMov
            orientation[3]=orientation[3]+rotMov
            np=sim.buildMatrix(position,orientation)
            sim.setObjectMatrix(objHandle,-1,sim.multiplyMatrices(np,sim.getObjectMatrix(objHandle,lineTracerBase)))
        end

        sim.switchThread() -- explicit thread switching: give control to other threads or the main thread
    end
end
