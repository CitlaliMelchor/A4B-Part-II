% MK2PLOT: Function that plots the MK2 robot given the transformation
%          matrices of each link.
%
%          function mk2plot(Tt,v);
%
%          Input:
%          Tt: Transformation matrices of the links: see robkin.m
%          v : argument for view command (optional)
%
%          Output: plot(s)
%
%          Note:
%          The transformation matrix Tt contains the transformation
%          matrices for all the links, i.e., 16 x 4 %
% 
%          See Introduction to Robotics Mechanics and Control,
%          J.J. Craig, 1989, Addison and Wesly, pp. 93-98, 136-141.
%          Observe the error on pp.98.
%


  function mk2plot(Tt,v)
% Tt=[eye(4),eye(4),eye(4),eye(4),eye(4)]; % Uncomment to see drawing
% without transformations
  
% Clear current figure
  figure(gcf); hold off; clf;

% Create new 3D figure with appropriate axes
% to plot the MK2 robot and hold this plot
  axis([-750 750 -750 750 -300 800]);
  xlabel('x'); ylabel('y'); zlabel('z');
  set(gca,'Clipping','off'); hold on;
  grid on
% Apply view if specified
  if nargin<2; v=[]; end
  if ~isempty(v); view(v); end
  
% FRAME
  O1 = [0,0,0,1];
  O2 = [0,0,0,1];
  O3 = [0,0,0,1];
  O4 = [0,0,0,1];
  O5 = [0,0,0,1];

%% Link 1 - BASE O1
% Coordinates of L1 with origin in O1
  L1 = [-125,125,125,-125,-125,125,125,-125;...
        125,125,-125,-125,125,125,-125,-125;...
        -200,-200,-200,-200,-450,-450,-450,-450]; 
  beamplot(L1,'bs')

%% Link 2 - BODY O1
  plot3(O1(1),O1(2),O1(3),'or') % Visualize O1
% Coordinates of L2 with origin in O1
  L2 = [-125,125,125,-125,-125,125,125,-125;...
        92.5,92.5,-92.5,-92.5,92.5,92.5,-92.5,-92.5;...
        50,50,50,50,-200,-200,-200,-200];
% Increase the coordinates matrix in one row of ones  
  L2 = [L2;ones(1,8)]; 
% Apply the transformation matrix
  L2 =  Tt(:,1:4)*L2; 
  beamplot(L2,'rs')

%% Link 3 - UPPER ARM O2
  O2=Tt(:,1:4)*O2';
  plot3(O2(1),O2(2),O2(3),'oc') % Visualize O2
% Coordinates of L3 with origin in O2
  L3 =[-100,370,370,-100,-100,370,370,-100;...
       40,40,-40,-40,40,40,-40,-40;...
       -92.5,-92.5,-92.5,-92.5,-167.5,-167.5,-167.5,-167.5]; 
% Increase the coordinates matrix in one row of ones
  L3 = [L3;ones(1,8)];
% Apply the transformation matrix
  L3 = Tt(:,5:8)*L3; 
  beamplot(L3,'cs')

%% Link 4 - FOREARM O3
  O3 = Tt(:,5:8)*O3';
  plot3(O3(1),O3(2),O3(3),'om') % Visualize O3
% Coordinates of L4 with origin in O3
  L4 =[-50,280,280,-50,-50,280,280,-50;...
       40,40,-40,-40,40,40,-40,-40;...
       -30,-30,-30,-30,-90,-90,-90,-90];
% Increase the coordinates matrix in one row of ones
  L4 = [L4;ones(1,8)];
% Apply the transformation matrix
  L4 =  Tt(:,9:12)*L4;
  beamplot(L4,'ms')
 
%% Link 5 - FLANGE O4
  O4 = Tt(:,9:12)*O4';
  plot3(O4(1),O4(2),O4(3),'oy') % Visualize O4
% Coordinates of L5 with origin in O4
  L5 = [-40,40,40,-40,-40,40,40,-40;...
        50,50,-100,-100,50,50,-100,-100;...
        30,30,30,30,-30,-30,-30,-30];
% Increase the coordinates matrix in one row of ones 
  L5 = [L5;ones(1,8)]; 
% Apply the transformation matrix
  L5 =  Tt(:,13:16)*L5;
  beamplot(L5,'ys')
 
%% Link 6 - GRIPPER O5
% The gripper is created with three similar beams named L61, L62 and L63
  O5 = Tt(:,13:16)*O5';
  plot3(O5(1),O5(2),O5(3),'oy') % Visualize O5
% Coordinates of L6 with origin in O5  
  L61 = [-40, 40, 40, -40,-40, 40, 40, -40;...
         -30,-30,30,30,-30,-30,30,30;...
         175,175,175,175,100,100,100,100];
     
  L62 = [-100, -20, -20, -100,-100, -20, -20, -100;...
         -30,-30,30,30,-30,-30,30,30;...
         250,250,250,250,175,175,175,175];
     
  L63 = [20, 100, 100, 20,20, 100, 100, 20;...
         -30,-30,30,30,-30,-30,30,30;...
         250,250,250,250,175,175,175,175];     
     
% Increase the coordinates matrix in one row of ones
  L61 = [L61;ones(1,8)];   
  L62 = [L62;ones(1,8)];
  L63 = [L63;ones(1,8)];  
% Apply the transformation matrix
  L61 =  Tt(:,17:20)*L61;
  L62 =  Tt(:,17:20)*L62;
  L63 =  Tt(:,17:20)*L63;
  beamplot(L61,'ws')
  beamplot(L62,'ws')
  beamplot(L63,'ws')
  
% No longer hold the plot
  drawnow; 
  hold off