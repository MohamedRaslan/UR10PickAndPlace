function GoAndLeave_Cuboid( id, vrep, handles, iTable,XYZoffset)
%% MohamedRaslan .. Muhammed Mohhie
    % Get the Target 
    relativToRef = handles.base;
    [res , TargPos]= vrep.simxGetObjectPosition(id,handles.ur10Tables(iTable),...
                          relativToRef,vrep.simx_opmode_buffer);
    vrchk(vrep, res,true);
    [res , Targtheta]= vrep.simxGetObjectOrientation(id,handles.ur10Tables(iTable),...
                            relativToRef,vrep.simx_opmode_buffer);
    vrchk(vrep, res,true);

   RG2GripperOffset= [0 0 0.6]; 
   TotOffset=XYZoffset+RG2GripperOffset;
%% Get the arm to be above the Cuboid   
   %Calculate And make offset
       %Note that Targtheta is in radian not deg.   
        G=EulerZYX(Targtheta)*ROT('Z',pi/2)*ROT('X',pi);
        G(1:3, 4)= TargPos + TotOffset; % XYZ
        G=double(G); 
   %Get Target Joint Value 
        q =handles.ur10Robot.ikunc(G); %1*6 vector
   %Move
        moveFrame( id, vrep, G, handles.Target, relativToRef); %move Target Frame
         pause(0.5);
        MoveUR10Joints(id, vrep, handles, q );%Then move the Arm  
  
%% Approaching the cuboid From above  

   TotOffset(3)=RG2GripperOffset(3)-0.1;
% Get the arm to be above the Cuboid   
   %Calculate And make offset
       %Note that Targtheta is in radian not deg.   
        G=EulerZYX(Targtheta)*ROT('Z',pi/2)*ROT('X',pi);
        G(1:3, 4)= TargPos + TotOffset; % XYZ
        G=double(G); 
   %Get Target Joint Value 
        q=handles.ur10Robot.ikunc(G); %1*6 vector
   %Move
        moveFrame( id, vrep, G, handles.Target, relativToRef); %move Target Frame
        pause(1.5);
        MoveUR10Joints(id, vrep, handles, q );%Then move the Arm 
        

%% If Reached .. Do Something    
  % open The Gripper
    openGripper(id,vrep,handles,0.1);
    pause(0.5);
  
%% Set the arm to its starting configuration
    
   RG2GripperOffset= [0 0 0.6]; 
   TotOffset=XYZoffset+RG2GripperOffset; 
   %Calculate And make offset
       %Note that Targtheta is in radian not deg.   
        G=EulerZYX(Targtheta)*ROT('Z',pi/2)*ROT('X',pi);
        G(1:3, 4)= TargPos + TotOffset; % XYZ
        G=double(G); 
   %Get Target Joint Value 
        q =handles.ur10Robot.ikunc(G); %1*6 vector
   %Move
        moveFrame( id, vrep, G, handles.Target, relativToRef); %move Target Frame
         pause(0.5);
        MoveUR10Joints(id, vrep, handles, q );%Then move the Arm  

        
%         res = vrep.simxPauseCommunication(id, true);
%         vrchk(vrep, res);
    for j = 1:6
            vrep.simxSetJointTargetPosition(id, handles.ur10Joints(j),...
                  handles.startingJoints(j),vrep.simx_opmode_oneshot);
            vrchk(vrep, res);
    end
%         res = vrep.simxPauseCommunication(id, false);
%         vrchk(vrep, res);
        
 pause(1); 
         
end