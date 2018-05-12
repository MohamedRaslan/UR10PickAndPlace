function openGripper(id,vrep,handles,Xspeed)
%% MohamedRaslan .. Muhammed Mohhie 
 %Open Gripper
 res=vrep.simxSetJointForce(id,handles.EndGripper,150,vrep.simx_opmode_oneshot);
 vrchk(vrep, res, true);
 res=vrep.simxSetJointTargetVelocity(id,handles.EndGripper,Xspeed,vrep.simx_opmode_oneshot);
 vrchk(vrep, res, true);
 pause(2.5);
 %Then stop
 res=vrep.simxSetJointTargetVelocity(id,handles.EndGripper,0,vrep.simx_opmode_oneshot);
 vrchk(vrep, res, true);
 

end

