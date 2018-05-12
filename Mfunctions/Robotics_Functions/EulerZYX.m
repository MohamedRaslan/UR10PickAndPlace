function [ rot_mat ] = EulerZYX( theta )
%% MohamedRaslan .. Muhammed Mohhie
    rot_mat = ROT('X',theta(1)) * ROT('Y',theta(2)) * ROT('Z',theta(3));
end