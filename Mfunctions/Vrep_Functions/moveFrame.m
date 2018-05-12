function moveFrame( id, vrep, g, h, hrel )
%% MohamedRaslan .. Muhammed Mohhie

    % Get the matrix from the 
    rot_mat = g(1:3, 1:3);
    pos = g(1:3, 4); 
    % Get theta
    theta =EulerZYX_inv(rot_mat);   
    % Set the Orientation &position
    res = vrep.simxSetObjectOrientation(id, h, hrel, theta, vrep.simx_opmode_oneshot);
    vrchk(vrep, res, true);
  
    res = vrep.simxSetObjectPosition(id, h, hrel, pos, vrep.simx_opmode_oneshot);
    vrchk(vrep, res, true);
end

