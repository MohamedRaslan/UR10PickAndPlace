%% Info.
        % Pick and Place.m
        % Author: Mohamed aly Ibrahim Raslan & Muhammed Mohhie El Dein Muhammed Fathy 
        % Robotics Course
        %       4rd Mechatronics Engineering 
        % Faculty Of Engineering Ain Shams University
        
%%% Note:
    % You Need to install Matlab Robotics Toolbox by Peter Corke
    % URL:['http://petercorke.com/wordpress/toolboxes/robotics-toolbox']
    % Or you can Simply(From within the MATLAB file browser double click on 
    % The file(Robotics Toolbox for MATLAB.mltbx) ,it will install and configure
    %          the paths correctly) ... You can find this file
    %          inside(RoboticsToolbox_installation)Folder

%% Iclude Path and M.Files 
addpath('..');
addpath('../VrepConnection','../Mfunctions/Robotics_Functions',...
    '../Mfunctions/URn_Functions','../Mfunctions/Vrep_Functions');
  
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
id=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);

if (id < 0)
    disp('Failed connecting to remote API server. Exiting.');
    vrep.delete();
    return;
end

fprintf('Connection %d to remote API server open.\n', id);

%% GetObjectHandle 

%%% For the UR10 Joints 
handles = struct('id',id);
jointNames={'UR10_joint1','UR10_joint2','UR10_joint3','UR10_joint4',...
    'UR10_joint5','UR10_joint6'};

handles.ur10Joints = -ones(1,6); 
for i = 1:6
    [res, handles.ur10Joints(i)] = vrep.simxGetObjectHandle(id, ...
                     jointNames{i}, vrep.simx_opmode_oneshot_wait); 
    vrchk(vrep, res);
end


%%% For the Cuboids & Tables
CuboidNames={'Left_Cuboid','Front_Cuboid','Right_Cuboid'};

TableNames={'Left_Table','Front_Table','Right_Table'};

handles.ur10Cuboids = -ones(1,3); 
handles.ur10Tables = -ones(1,3); 

 for i = 1:3
    [res, handles.ur10Cuboids(i)] = vrep.simxGetObjectHandle(id,...
                    CuboidNames{i}, vrep.simx_opmode_oneshot_wait); 
    vrchk(vrep, res);
    
    [res, handles.ur10Tables(i)] = vrep.simxGetObjectHandle(id,...
                    TableNames{i}, vrep.simx_opmode_oneshot_wait); 
    vrchk(vrep, res);       
end

%%% For the Ref & Gripper
  % Those Handle Not Used in the Simulation 
handles.ur10Ref= -1;
handles.ur10Gripper=-1;
[res, handles.ur10Ref] = vrep.simxGetObjectHandle(id, 'UR10', ...
    vrep.simx_opmode_oneshot_wait); 
vrchk(vrep, res);

[res, handles.ur10Gripper] = vrep.simxGetObjectHandle(id, 'UR10_connection', ...
    vrep.simx_opmode_oneshot_wait);
vrchk(vrep, res);

%%% For the Base frame [Frame0]
handles.base=-1;
[res, handles.base] = vrep.simxGetObjectHandle(id, ...
    'Frame0', vrep.simx_opmode_oneshot_wait);
vrchk(vrep, res);
%%% For the Target frame [Frame1]
handles.Target=-1;
[res, handles.Target] = vrep.simxGetObjectHandle(id, ...
    'Frame1', vrep.simx_opmode_oneshot_wait);
vrchk(vrep, res);

%%% For the EndGripper And AngEndGeipperJoints
handles.EndGripper= -1 ;
    [res, handles.EndGripper] = vrep.simxGetObjectHandle(id, ...
                  'RG2_openCloseJoint', vrep.simx_opmode_oneshot_wait); 
    vrchk(vrep, res);

%% Stream ...

%%% Cuboids [position/orientation & Tables [position/orientation]
relativToRef = handles.base;
for i = 1:3
    
    %Cuboids
    res = vrep.simxGetObjectPosition(id,handles.ur10Cuboids(i),relativToRef,...
               vrep.simx_opmode_streaming);
    vrchk(vrep, res, true);
    res = vrep.simxGetObjectOrientation(id,handles.ur10Cuboids(i),relativToRef,...
               vrep.simx_opmode_streaming);
    vrchk(vrep, res, true);  
    
    % Tables
    res = vrep.simxGetObjectPosition(id,handles.ur10Tables(i),relativToRef,...
               vrep.simx_opmode_streaming);
    vrchk(vrep, res, true);
    res = vrep.simxGetObjectOrientation(id,handles.ur10Tables(i),relativToRef,...
               vrep.simx_opmode_streaming);
    vrchk(vrep, res, true);  
end

%%% ur10Gripper[position/orientation] & ur10Ref [position/orientation]
% No need for them here
%{
relativToRef = handles.base;
    res = vrep.simxGetObjectPosition(id, handles.ur10Ref,relativToRef,...
               vrep.simx_opmode_streaming); 
    vrchk(vrep, res, true);
    res = vrep.simxGetObjectOrientation(id, handles.ur10Ref,relativToRef,...
               vrep.simx_opmode_streaming); 
    vrchk(vrep, res, true);

relativToRef = handles.ur10Ref;
    res = vrep.simxGetObjectPosition(id, handles.ur10Gripper,relativToRef,...
               vrep.simx_opmode_streaming);
    vrchk(vrep, res, true);
    res = vrep.simxGetObjectOrientation(id, handles.ur10Gripper,relativToRef,...
               vrep.simx_opmode_streaming);
    vrchk(vrep, res, true);
%}

%%% EndGripper & AngEndGeipperJoints    
  res = vrep.simxGetJointPosition(id,handles.EndGripper,...
             vrep.simx_opmode_streaming); 
  vrchk(vrep, res, true);
    
  



%%%% The UR10 Joints
for i = 1:6
    res = vrep.simxGetJointPosition(id, handles.ur10Joints(i),...
               vrep.simx_opmode_streaming); 
    vrchk(vrep, res, true);
end


%% Start
% to make sure that streaming data has reached to client at least once
vrep.simxGetPingTime(id);
vrep.simxStartSimulation(id, vrep.simx_opmode_oneshot_wait);

%% Simulation

% Set the threshold to check if the end effector has reached its destination
  handles.threshold = 0.01;
%Set The Arm Parameters Using Peter Corke robotics toolbox
handles.ur10Robot= URnSerial_fwdtrans('UR10');
pause(0.5);
%Rest Joint for 1st time
handles.startingJoints = [0,0,0, 0, 0, 0];
        res = vrep.simxPauseCommunication(id, true);
        vrchk(vrep, res);
    for j = 1:6
            vrep.simxSetJointTargetPosition(id, handles.ur10Joints(j),...
                  handles.startingJoints(j),vrep.simx_opmode_oneshot);
            vrchk(vrep, res);
    end
        res = vrep.simxPauseCommunication(id, false);
        vrchk(vrep, res);
pause(1);
 
% Here's Where the Fun Begins!... 
ConstValue=0.25;
iTable=1;  % 1 for 'Left_Table', 2 for'Front_Table', 3 for'Right_Table'
iCuboid=1; % 1 for 'Left_Cuboid', 2 for'Front_Cuboid', 3 for'Right_Cuboid'
XYZoffset= [0 0 0]; % [ X Y Z]
%%% 1st Sequence

   iCuboid=1;  XYZoffset=[0 0 ConstValue];
 GoAndCatch_Cuboid( id, vrep, handles, iCuboid,XYZoffset);
   iTable=1;   XYZoffset=[0 ConstValue 0];
 GoAndLeave_Cuboid( id, vrep, handles, iTable ,XYZoffset);
 
   iCuboid=2;  XYZoffset=[0 0 ConstValue];
 GoAndCatch_Cuboid( id, vrep, handles, iCuboid,XYZoffset);
   iTable=2;   XYZoffset=[0 ConstValue 0];
 GoAndLeave_Cuboid( id, vrep, handles, iTable ,XYZoffset);
 
   iCuboid=3;  XYZoffset=[0 0 ConstValue];
 GoAndCatch_Cuboid( id, vrep, handles, iCuboid,XYZoffset);
   iTable=3;   XYZoffset=[0 -ConstValue 0];
 GoAndLeave_Cuboid( id, vrep, handles, iTable ,XYZoffset);
  
% Back Where We Started From
g= eye(4,4);
g(1:3,4)=[-0.25 1 0.7301];
moveFrame( id, vrep, g, handles.ur10Cuboids(1), handles.base );
g(1:3,4)=[0 1 0.7301];
moveFrame( id, vrep, g, handles.ur10Cuboids(2), handles.base );
g(1:3,4)=[0.25 1 0.7301];
moveFrame( id, vrep, g, handles.ur10Cuboids(3), handles.base ); 
pause(2);
   
%%% 2nd Sequence
for i=1:3
    
   iCuboid=i;  XYZoffset=[0 0 ConstValue];
 GoAndCatch_Cuboid( id, vrep, handles, iCuboid,XYZoffset);
   iTable=1;   XYZoffset=[0 0 0.15*i];
 GoAndLeave_Cuboid( id, vrep, handles, iTable ,XYZoffset);

end


%% END ..
vrep.delete;
clear; clc;


