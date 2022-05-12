% MK2PLOT: Plot MK2 robot configuration given the transformation
%          matrices of each link and its size parameters
%
%          function mk2plot(Tt,v);
%
%          Tt: Transformation matrices of the links: see robkin.m
%          v : argument for view command (optional)
%
%          See Introduction to Robotics Mechanics and Control,
%          J.J. Craig, 1989, Addison and Wesly, pp. 93-98, 136-141.
%          Observe the error on pp.98.
%
%          L.G. van Willigenburg 23-5-'96
%
  function mk2plot(Tt,v)

  % Clear current figure
  figure(gcf); hold off; clf;

  % Create new 3D figure with appropriate axes
  % to plot the MK2 robot and hold this plot
  axis([-750 750 -750 750 -300 1000]);
  xlabel('x'); ylabel('y'); zlabel('z');
  set(gca,'Clipping','off'); hold on;

  % Apply view if specified
  if nargin<2; v=[]; end;
  if ~isempty(v); view(v); end;

  sc=62.5; % Scale factor
  
% Base 0
% Specify the corners of the box that approximates the link
  e1=[-2.5;-1.5;-7]; e2=[1.5;-1.5;-7];
  e3=[1.5;1.5;-7]  ; e4=[-2.5;1.5;-7];
  eh=[0;0;4]; e5=e1+eh; e6=e2+eh; e7=e3+eh; e8=e4+eh;
  T=eye(4); %T(0,N)
% Plot beam that approximates the link
  beamplot(T*[sc*[e1 e2 e3 e4 e5 e6 e7 e8]; ones(1,8)],'rs');

% Link 1
% Specify the corners of the box that approximates the link
  e1=[-2.5;-1.5;-3]; e2=[1.5;-1.5;-3];
  e3=[1.5;1.5;-3]  ; e4=[-2.5;1.5;-3];
  eh=[0;0;6]; e5=e1+eh; e6=e2+eh; e7=e3+eh; e8=e4+eh;
  T=Tt(:,1:4); %T(0,N)
% Plot beam that approximates the link
  beamplot(T*[sc*[e1 e2 e3 e4 e5 e6 e7 e8]; ones(1,8)],'gs');

% Link 2
% Specify the corners of the box that approximates the link
  e1=[-1.5;1;-0.8]; e2=[5;1;-0.8];
  e3=[5;1;0.8]    ; e4=[-1.5;1;0.8];
  eh=[0;-2;0]; e5=e1+eh; e6=e2+eh; e7=e3+eh; e8=e4+eh;
  T=Tt(:,5:8); %T(0,N)
% Plot beam that approximates the link
  beamplot(T*[sc*[e1 e2 e3 e4 e5 e6 e7 e8]; ones(1,8)],'bs');

% Link 3
% Specify the corners of the box that approximates the link
% Use the transformation matrix of this link and plot the box
  e1=[-0.75;0.75;-0.5]; e2=[4.5;0.75;-0.5];
  e3=[4.5;0.75;0.5]   ; e4=[-0.75;0.75;0.5];
  eh=[0;-1.5;0]; e5=e1+eh; e6=e2+eh; e7=e3+eh; e8=e4+eh;
  T=Tt(:,9:12); %T(0,N)
% Plot beam that approximates the link
  beamplot(T*[sc*[e1 e2 e3 e4 e5 e6 e7 e8]; ones(1,8)],'ys');
  
% Link 4
% Specify the corners of the box that approximates the link
%!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
%!!! Link 4 points UPWARDS for zero rotation
%!!! due to the definition of theta4 :
%!!! For theta4=0 X3 and X4 point in the same direction
%!!! Notice the ERROR in Craig (corrected in notes)
%!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
% Specify the corners of the box that approximates the link
% Use the transformation matrix of this link and plot the box
  e1=[0.6;1.0;-0.5]  ; e2=[0.6;-1.5;-0.5];
  e3=[0.6;-1.5;0.5]  ; e4=[0.6;1.0;0.5];
  eh=[-1.2;0;0]; e5=e1+eh; e6=e2+eh; e7=e3+eh; e8=e4+eh;
  T=Tt(:,13:16); %T(0,N)
% Plot beam that approximates the link
  beamplot(T*[sc*[e1 e2 e3 e4 e5 e6 e7 e8]; ones(1,8)],'ks');
  
% Link 5 the gripper
%!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
% !!! See remarks link 4
%!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  e1=[0.5;-0.5;1.5]; e2=[0.5;-0.5;2.5];
  e3=[0.5;0.5;2.5] ; e4=[0.5;0.5;1.5];
  eh=[-1;0;0]; e5=e1+eh; e6=e2+eh; e7=e3+eh; e8=e4+eh;
  T=Tt(:,17:20); %T(0,N)  
% Plot beam that approximates the link  
  beamplot(T*[sc*[e1 e2 e3 e4 e5 e6 e7 e8]; ones(1,8)],'ws');

% Translated gripper block
  T1=[eye(3) sc*[-0.75; 0; 1]; 0 0 0 1];
% Plot beam that approximates the link    
  beamplot(T*T1*[sc*[e1 e2 e3 e4 e5 e6 e7 e8]; ones(1,8)],'ws');
  
% Translated gripper block  
  T1=[eye(3) sc*[0.75; 0; 1]; 0 0 0 1];
% Plot beam that approximates the link  
  beamplot(T*T1*[sc*[e1 e2 e3 e4 e5 e6 e7 e8]; ones(1,8)],'ws');  

  hold off