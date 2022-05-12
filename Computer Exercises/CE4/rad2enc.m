% RAD2ENC : function converts radians to encodercounts of the MK2 robot
%
%           [thenc]=rad2enc(thetat)
%
%   Input:
%   thetat: [theta1; theta2; theta3; theta4; theta5]
%           matrix which contains the joint angles in radians
%           the first row the successive values of theta1,
%           the second row the successive values of
%           theta2 etc.
%
%   Output:
%   thenc: joint angles of the robot in encoder counts
%          the first row the encodercounts of successive values of theta1,
%          the second row the encodercounts of successive values of
%          theta2 etc.
%
% GvW 3-8-2008

  function [thenc]=rad2enc(thetat)
  
  % Check dimension thetat
  [n,m]=size(thetat);
  if n~=5; error('thetat should me an 5xn matrix'); end;

  % Conversion from radians to degrees
  thetat=180/pi*thetat;
  
  % Initialize thenc
  thenc=thetat;
  
  % Conversion from degrees to encodercounts
  thenc(1,:)=   2388+60000/90*thetat(1,:);
  thenc(2,:)= -77596+80000/90*(-thetat(2,:));
  thenc(3,:)=   2371+72000/90*(-thetat(3,:));
  thenc(4,:)=   3104+60000/90*(-thetat(4,:)+90);
  thenc(5,:)=   4900+44000/90*thetat(5,:);