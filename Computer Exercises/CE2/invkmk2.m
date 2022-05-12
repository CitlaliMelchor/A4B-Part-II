% INVKMK2: Compute the inverse kinematic transformations
%          of the MK2 robot (adapted from Yasukawa)
%
%          function [th,th2,th3,th4,Ti,TMK2]=invkMK2(T,mf);
%
%          T : matrix specifying the transformation from
%              base to the TCP
%          mf: fixed part of matrix with robot kinematic parameters
%              mf=m(:,1:3) (first 3 columns of m)
%
%          th,th2,th3,th4: Vectors containing the joint angles
%              for each link (n = number of links) for each solution.
%          Ti: Transformation matrix used to compare the 
%              the inverse kinematics with the forward kinematics
%          TMK2: Transformation matrix in the MK2 workspace
%                closest to T
%
% 
  function [th,Ti,TMK2]=invkmk2(T,mf)

% Check if the input is a transformation matrix 
% and generate an error message if not
  if istran(T)
    TMK2=T; th=zeros(5,1); % Copy T to TMK2, initialize th
  else
    error('INVKMK2: The input is not a transformation matrix');
  end

% Compute transformation into MK2 workspace
  xhat=T(1:3,1); yhat=T(1:3,2); zhat=T(1:3,3);
  pxMK2=T(1,4); pyMK2=T(2,4); pz=T(3,4);
  
% Calculate variables for the inverse kinematics
  m=[pyMK2;-pxMK2;0]; mhat=m/norm(m);
  khat= cross(mhat,zhat)/ norm(cross(mhat,zhat));
  zhatp=cross(khat,mhat);
  ct=zhat'*zhatp; st=cross(zhat,zhatp)'*khat;
  yhatp= ct*yhat+st*cross(khat,yhat)+(1-ct)*(dot(khat,yhat)*khat);
  xhatp= cross(yhatp,zhatp);

% Create the transformation matrix
  TMK2(1:3,1)=xhatp;
  TMK2(1:3,2)=yhatp;
  TMK2(1:3,3)=zhatp;

% Check if the transformation is active or not    
  if norm(T-TMK2)>1e-3
    disp('INVKMK2: MK2 subspace transformation active');
  end

% Compute inverse kinematics
  r11=TMK2(1,1); r12=TMK2(1,2);  r13=TMK2(1,3); pxMK2=TMK2(1,4); 
  r21=TMK2(2,1); r22=TMK2(2,2);  r23=TMK2(2,3); pyMK2=TMK2(2,4); 
  r31=TMK2(3,1); r32=TMK2(3,2);  r33=TMK2(3,3); pz=TMK2(3,4);

% Get l1, l2, l3 from mf  
  l1 = mf(2,2); % yasukawa motoman l1 = 0
  l2 = mf(3,2);
  l3 = mf(4,2);

%% Solution 1
% Initialize th
  th=zeros(5,1);

% Compute th(1);
  th(1)=atan2(pyMK2,pxMK2); c1=cos(th(1)); s1=sin(th(1));

% Adaptation from MK2 to Yasukawa inverse kinematics (l1~=0 for MK2)
  px=pxMK2-c1*l1; py=pyMK2-s1*l1; 

% Compute th234=th(2)+th(3)+th(4)
  th234=atan2(c1*r13+s1*r23,r33);

% Compute cos(th3), sin(th3), phi and beta
  c3 = (px^2+py^2+pz^2-l2^2-l3^2)/(2*l2*l3);
  s3 = sqrt(1-c3^2);
  phi= atan2(pz, sqrt(px^2+py^2));
  beta= atan2(l3*s3, l2+l3*c3);
  
% Compute th(2), th(3), th(4), th(5)
  th(2)= -(phi+beta);
  th(3)= atan2(s3,c3);
  th(5)= atan2(-s1*r11+c1*r21, -s1*r12+c1*r22);
  th(4)= th234-th(2)-th(3);
  
% Check the inverse kinematics by recomputing Ti
% the forward kinematics transformation of the TCP
% obtained from th(1)-th(5) of the inverse kinematics
   Ti = robkin([mf(:,1:3),th]);
   Ti = Ti(:,17:20);
  
% Plot message if inverse and forward kinematics do not match
  if norm(Ti-TMK2)>1e-3
    disp('INVKMK2 solution 1:');
    disp('The inverse kinematics do not match the forward kinematics');
  end
  
%% Solution 2, not compulsory
% Initialize the th2 vector
  th2 = zeros(5,1);
  
% holds for the same theta 1
  th2(1) = th(1);
  
% calculate the other thetas
  th2(2)= -(phi-beta);
  
% take the negative th3
  th2(3) = -th(3);
  
% calculate th4 with the new th2 and th3
  th2(4) = th234-th2(2)-th2(3);

% th5 still holds
  th2(5) = th(5);

%% Uncomment to check solutions 3 and 4 with T4 matrix
% %% Solution 3, not compulsory
% % Initialize the th3 vector
%   th3 = zeros(5,1);
%   
% % Add pi to get new th3
%   th3(1) = th(1)+pi;
%   
% % Compute c1p and s1p from the new th1
%   c1p=cos(th3(1)); s1p=sin(th3(1));
% 
% % Adaptation from MK2 to Yasukawa inverse kinematics (l1~=0 for MK2)
%   pxp=pxMK2-c1p*l1; pyp=pyMK2-s1p*l1; 
%   
% % calculate c3p and s3p
%   c3p = (pxp^2+pyp^2+pz^2-l2^2-l3^2)/(2*l2*l3);
%   s3p = sqrt(1-c3p^2);
%   
% % calculate phip and betap and th2
%   phip = pi - atan2(pz, sqrt(pxp^2+pyp^2));
%   betap = atan2(l3*s3p, l2+l3*c3p);
%   th3(2) = -(phip+betap);
%     
% % calculate th3 from the s3p and c3p
%   th3(3) = atan2(s3p,c3p);
%   
% % first calculate th234p, then calculate th3(4)
%   th234p=atan2(c1p*r13+s1p*r23,r33);
%   th3(4) = th234p - th3(2)-th3(3);
%   
% % substract pi from previous th5
%   th3(5) = th(5)-pi;
% 
% %% Solution 4, not compulsory
% % Initialize th4
%   th4 = zeros(5,1);
%   
% % th1 still holds
%   th4(1) = th3(1);
%   
% % Use the second method to calculate th4
%   th4(2) = -(phip-betap);
%     
% % take the negative value for th3
%   th4(3) = -th3(3);
%   
% % first calculate th234p, then calculate th3(4)
%   th4(4) = th234p - th4(2)-th4(3);
%   
% % th5 still holds
%   th4(5) = th3(5);