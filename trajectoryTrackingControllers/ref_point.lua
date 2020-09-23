function sysCall_threadmain()
    -- Put some initialization code here
path=sim.getObjectHandle("Path")


    -- Put your main loop here, e.g.:
 --solution from https://forum.coppeliarobotics.com/viewtopic.php?t=4475
 --and https://forum.coppeliarobotics.com/viewtopic.php?f=9&t=4247
 
    moveToPosition=function(_pathHandle,startPos,endPos,maxVel,maxAccel,maxJerk)
	local posVelAccel={startPos,0,0}
	local targetPosVel={endPos,0}
	local maxVelAccelJerk={maxVel,maxAccel,maxJerk}
	local rmlHandle=sim.rmlPos(1,0.0001,-1,posVelAccel,maxVelAccelJerk,{1},targetPosVel)
	local newPosVelAccel=posVelAccel
	while true do
		local result,newPosVelAccel=sim.rmlStep(rmlHandle,sim.getSimulationTimeStep())
		sim.setPathPosition(_pathHandle,newPosVelAccel[1]) -- update the path's intrinsic position
		if result~=0 then
			break
		end
		sim.switchThread()

	end
	sim.rmlRemove(rmlHandle)
    
end
moveToPosition(path,0,30,0.1,0.01,0.01)

end

function sysCall_cleanup()
    -- Put some clean-up code here
end

-- See the user manual or the available code snippets for additional callback functions and details
