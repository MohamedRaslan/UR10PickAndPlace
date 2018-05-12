function [ G ] = URnSerial_fwdtrans( URnName )
%% MohamedRaslan .. Muhammed Mohhie
%Creat The Arm  Using Peter Corke robotics toolbox
  switch URnName
   case 'UR3'
     a = [0, -0.24365, -0.21325, 0, 0, 0];
     d = [0.1519, 0, 0, 0.11235, 0.08535, 0.0819];
     alpha = [1.570796327, 0, 0, 1.570796327, -1.570796327, 0];
   case 'UR5'
      a =[0, -0.42500, -0.39225, 0, 0, 0];
      d =[0.089459, 0, 0, 0.10915, 0.09465, 0.0823];
      alpha =[1.570796327, 0, 0, 1.570796327, -1.570796327, 0];
   case 'UR10'
      a = [0, -0.612, -0.5723, 0, 0, 0];
      d = [0.1273, 0, 0, 0.163941, 0.1157, 0.0922];
      alpha = [1.570796327, 0, 0, 1.570796327, -1.570796327, 0];
      otherwise
        a = [0, -0.612, -0.5723, 0, 0, 0];
        d = [0.1273, 0, 0, 0.163941, 0.1157, 0.0922];
        alpha = [1.570796327, 0, 0, 1.570796327, -1.570796327, 0];   
  end
   offset= [0, -pi/2, 0,-pi/2, 0, 0];
  for i= 1:6
      L(i) = Link([ 0 d(i) a(i) alpha(i) 0 offset(i)], 'standard');    
  end
    G = SerialLink(L,'name','RUn'); 
end


