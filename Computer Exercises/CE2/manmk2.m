function [th,pt,Tt,tht]=manmk2(th,pt,Tt,tht)
% MANMK2 : Function to plot and manually move the MK2 robot by key strokes
%          It also provides recording and storing facilities
%
%          Possible calls:
%          manmk2 (stores th,pt,Tt,tht in matlab file manmk2.mat)
%          [th,pt,Tt,tht]=manmk2
%          [th,pt,Tt,tht]=manmk2(th,pt,Tt,tht)
%
%          Inputs : Note that all inputs are optional so need not be specified 
%          th  : columnvector of 5 starting joint angles (optional)
%                default [0;0;0;90;0]*pi/180;
%          pt  : Stored TCP positions (optional)
%                default []
%          Tt  : Stored TCP transformation matrices (optional)
%                default []
%          tht : Stored joint angles (optional)
%                default []
%      
%          Outputs:
%          th  : columnvector of 5 joint angles after stop
%          pt  : Stored TCP positions
%          Tt  : Stored TCP transformation matrices
%          tht : Stored joint angles
%          file manmk2.mat that stores th,pt,Tt,tht
% 
%          Key strokes:
%          Type 1/q for up/down axis 1; 2/w for up/down axis 2 etc.;
%          Type z to toggle between steps of 10 and 1 degree rotation
%          Type c to store the TCP transf. matrix and position in Tt, pt
%          Type b to stop
%          !! Response to keys occurs only when the figure is selected !!
%
% GvW 15-8-2016

% Check inputs
if nargin<1; th=[0,0,0,90,0]'*pi/180; end;
if nargin<2; pt=[]; end
if nargin<3; Tt=[]; end
if nargin<4; tht=[]; end

% Initialize
disp(' ');
disp(' Manual rotation and plot MK2 robot axes')
disp(' Type 1/q, for up/down axis 1; 2/w for up/down axis 2 etc.');
disp(' Type z to toggle between steps of 10 and 1 degree rotation')
disp(' Type c to store the current transformation matrix')
disp(' Type b to stop')
disp(' !! Response to keys occurs only')
disp('    when the figure is selected !!')

s=1; sd=10; stp=0; % set stop flag, stepsize, store flag
close all; hf=figure; % create figure
set(hf,'units','normalized','outerposition',[0.5 0.05 0.5 0.95]);
title('Type: b to stop, c to store, z to toggle angle stepsize')

% Fixed part of matrix with robot kinematic parameters  
mf=[   0     0  0;
    -0.5*pi 100 -130;
       0    270 75;
       0    230 55;
     0.5*pi  0  0];

% Maximum and minimum angles
thmin=[-152.16 -156.58 -118.78 -113.12+90 -90]'*pi/180;
thmax=[ 154.30   38.19  113.02  113.48+90  90]'*pi/180;

while s % Input loop
  % Plot robot, display joint angles and TCP position
  T=robkin([mf,th]); mk2plot(T,[15,45]);
  title('Type: b to stop, c to store, z to toggle angle stepsize, 1/q to rotate axis 1, 2/w to rotate axis 2 etc.')
  if ~stp
    disp(' TCP position:'); disp(T(1:3,20))
    disp(' Joint angles:'); disp(th);
  end
 [dum,dum,butp]=ginput(1); stp=0; % wait for key stroke
  switch butp % handle keystroke
    case 49 % 1; up th(1)
      th(1)=th(1)+sd*pi/180;
    case 50 % 2; up th(2)
      th(2)=th(2)+sd*pi/180;
    case 51 % 3; up th(3)
      th(3)=th(3)+sd*pi/180;
    case 52 %4; up th(4)
      th(4)=th(4)+sd*pi/180;
    case 53 %5; up th(5)
      th(5)=th(5)+sd*pi/180;
    case 113 % q; down th(1)
      th(1)=th(1)-sd*pi/180;
    case 119 % w; down th(2)
      th(2)=th(2)-sd*pi/180;
    case 101 % e; down th(3)
      th(3)=th(3)-sd*pi/180;
    case 114 % r; down th(4)
      th(4)=th(4)-sd*pi/180;
    case 116 % t; down th(5)
      th(5)=th(5)-sd*pi/180;
    case 98 % b; end manmk2
      s=0; disp(' manmk2 ended');
    case 99 % c; Store T in Tt
      Tt=[Tt,T(:,17:20)]; pt=[pt,T(1:3,20)]; tht=[tht,th];
      disp(' Stored TCP position:'); disp(pt(:,end)); stp=1;
    case 122 % z; toggle angle stepsize between 1 and 10
      if sd==10; 
        sd=1; disp(' Angle stepsize changed to 1')
      else sd=10; disp(' Angle stepsize changed to 10');
      end
    otherwise % erroneous input
      disp('erroneous input')
  end
  th=max(th,thmin); % maximize
  th=min(th,thmax); % minimize
end
save manmk2 th pt Tt tht % save stored positions and transformation matrices