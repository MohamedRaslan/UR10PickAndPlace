function MoveUR10Joints(id, vrep, handles, Theta )
%% MohamedRaslan .. Muhammed Mohhie
% Make it within range
            targetTheta=Theta+handles.startingJoints;

    for i= 1:6
        if targetTheta(i) > pi
            targetTheta(i) = targetTheta(i) - 2*pi;
        elseif targetTheta(i) < -pi
            targetTheta(i) = targetTheta(i) + 2*pi;
        end
    end 
% Adjust jiont0 angle
    targetTheta(1)=targetTheta(1)+(pi/2);
% Act
    for j = 1:6
            res =vrep.simxSetJointTargetPosition(id, handles.ur10Joints(j),...
                  targetTheta(j),vrep.simx_opmode_oneshot);
            vrchk(vrep,res,true);
    end

%Check that action 
        currentTheta = zeros(1,6);    

        while true                        
        % Get current joint angles for each joint
            for i = 1:6
                [res,currentTheta(i)] = vrep.simxGetJointPosition(id, handles.ur10Joints(i),...
                                         vrep.simx_opmode_oneshot_wait);
                vrchk(vrep, res, true);
            end  
            % Check joints 
            diffJoints = currentTheta - targetTheta;
            if max(abs(diffJoints)) < handles.threshold
               break; 
            end
        end

end