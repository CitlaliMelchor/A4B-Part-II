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
  function [a] = mk2plot(Tt,v)
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
  rs = 50; % Sphere radius
  
%% Create the robot environment
  position = [-300,700,-437.5]/sc;
  
% Create stem of the tree
% Specify the corners of the box that approximates the stem
  e1 = [position(1)-1; position(2)-1; position(3)];
  e2 = [position(1)+1; position(2)-1; position(3)];
  e3 = [position(1)+1; position(2)+1; position(3)];
  e4 = [position(1)-1; position(2)+1; position(3)];
  eh=[0;0;20]; e5=e1+eh; e6=e2+eh; e7=e3+eh; e8=e4+eh;
  
% Plot tree stem 
  coord = [sc*[e1 e2 e3 e4 e5 e6 e7 e8]; ones(1,8)];
  beamplot(coord,'ys');
  T = eye(4);
  stem_centers = beam2sphere(sc*[e1 e2 e3 e4 e5 e6 e7 e8], rs, '#663300',T);
% Create branches/leaves of the tree
% Specify the corners of the box that approximates the top of the tree
  e1 = [position(1)-5; position(2)-5; position(3)+20];
  e2 = [position(1)+5; position(2)-5; position(3)+20];
  e3 = [position(1)+5; position(2)+5; position(3)+20];
  e4 = [position(1)-5; position(2)+5; position(3)+20];
  eh=[0;0;8]; e5=e1+eh; e6=e2+eh; e7=e3+eh; e8=e4+eh;
% Plot top of the tree
  coord = [sc*[e1 e2 e3 e4 e5 e6 e7 e8]; ones(1,8)];
  beamplot(coord,'gs');
  T = eye(4);
 % beam2sphere(sc*[e1 e2 e3 e4 e5 e6 e7 e8], size, 'g',T); % Hidden to
 % reduce computation time
  
 
%% Base 0
% Specify the corners of the box that approximates the link
  e1=[-2.5;-1.5;-7]; e2=[1.5;-1.5;-7];
  e3=[1.5;1.5;-7]  ; e4=[-2.5;1.5;-7];
  eh=[0;0;4]; e5=e1+eh; e6=e2+eh; e7=e3+eh; e8=e4+eh;
  T=eye(4); %T(0,N)
% Plot beam that approximates the link
  coord = T*[sc*[e1 e2 e3 e4 e5 e6 e7 e8]; ones(1,8)];
  beamplot(coord,'rs');
% Plot the spheres
  %beam2sphere(sc*[e1 e2 e3 e4 e5 e6 e7 e8], size,'#D95319',T);% Hidden to
 % reduce computation time 
  
  
%% Link 1
% Specify the corners of the box that approximates the link
  e1=[-2.5;-1.5;-3]; e2=[1.5;-1.5;-3];
  e3=[1.5;1.5;-3]  ; e4=[-2.5;1.5;-3];
  eh=[0;0;6]; e5=e1+eh; e6=e2+eh; e7=e3+eh; e8=e4+eh;
  T=Tt(:,1:4); %T(0,N)
% Plot beam that approximates the link
  coord = T*[sc*[e1 e2 e3 e4 e5 e6 e7 e8]; ones(1,8)];
  beamplot(coord,'gs');
% Plot the spheres
%  beam2sphere(sc*[e1 e2 e3 e4 e5 e6 e7 e8], size, '#77AC30',T);% Hidden to
 % reduce computation time
%%  
% Link 2
% Specify the corners of the box that approximates the link
  e1=[-1.5;1;-0.8]; e2=[5;1;-0.8];
  e3=[5;1;0.8]    ; e4=[-1.5;1;0.8];
  eh=[0;-2;0]; e5=e1+eh; e6=e2+eh; e7=e3+eh; e8=e4+eh;
  T=Tt(:,5:8); %T(0,N)
% Plot beam that approximates the link
  coord = T*[sc*[e1 e2 e3 e4 e5 e6 e7 e8]; ones(1,8)];
  beamplot(coord,'bs');
% Plot the spheres
%  beam2sphere(sc*[e1 e2 e3 e4 e5 e6 e7 e8], size,'#0072BD',T);% Hidden to
 % reduce computation time
  
%% Link 3
% Specify the corners of the box that approximates the link
% Use the transformation matrix of this link and plot the box
  e1=[-0.75;0.75;-0.5]; e2=[4.5;0.75;-0.5];
  e3=[4.5;0.75;0.5]   ; e4=[-0.75;0.75;0.5];
  eh=[0;-1.5;0]; e5=e1+eh; e6=e2+eh; e7=e3+eh; e8=e4+eh;
  T=Tt(:,9:12); %T(0,N)
% Plot beam that approximates the link
  coord = T*[sc*[e1 e2 e3 e4 e5 e6 e7 e8]; ones(1,8)];
  beamplot(coord,'ys');
% Plot the spheres
  s3_centers = beam2sphere(sc*[e1 e2 e3 e4 e5 e6 e7 e8], rs, '#EDB120',T);
%% Link 4
% Specify the corners of the box that approximates the link
% Use the transformation matrix of this link and plot the box
  e1=[0.6;1.0;-0.5]  ; e2=[0.6;-1.5;-0.5];
  e3=[0.6;-1.5;0.5]  ; e4=[0.6;1.0;0.5];
  eh=[-1.2;0;0]; e5=e1+eh; e6=e2+eh; e7=e3+eh; e8=e4+eh;
  T=Tt(:,13:16); %T(0,N)
% Plot beam that approximates the link
  coord = T*[sc*[e1 e2 e3 e4 e5 e6 e7 e8]; ones(1,8)];
  beamplot(coord,'ks');
% Plot the spheres
  s4_centers = beam2sphere(sc*[e1 e2 e3 e4 e5 e6 e7 e8], rs,'#A9A9A9',T);
    
%% Link 5 the gripper
  e1=[0.5;-0.5;1.5]; e2=[0.5;-0.5;2.5];
  e3=[0.5;0.5;2.5] ; e4=[0.5;0.5;1.5];
  eh=[-1;0;0]; e5=e1+eh; e6=e2+eh; e7=e3+eh; e8=e4+eh;
  T=Tt(:,17:20); %T(0,N)  
% Plot beam that approximates the link  
  coord = T*[sc*[e1 e2 e3 e4 e5 e6 e7 e8]; ones(1,8)];
  beamplot(coord,'ws');
% Plot the spheres
  s60_centers = beam2sphere(sc*[e1 e2 e3 e4 e5 e6 e7 e8], rs,'#D3D3D3',T);

% Translated gripper block
  T1=[eye(3) sc*[-0.75; 0; 1]; 0 0 0 1];
% Plot beam that approximates the link    
  coord = T*T1*[sc*[e1 e2 e3 e4 e5 e6 e7 e8]; ones(1,8)];
  beamplot(coord,'ws');
% Plot the spheres
  s61_centers = beam2sphere(sc*[e1 e2 e3 e4 e5 e6 e7 e8], rs, '#D3D3D3',T*T1);
    
% Translated gripper block  
  T1=[eye(3) sc*[0.75; 0; 1]; 0 0 0 1];
% Plot beam that approximates the link  
  coord = T*T1*[sc*[e1 e2 e3 e4 e5 e6 e7 e8]; ones(1,8)];
  beamplot(coord,'ws');  
% Plot the spheres
  s62_centers = beam2sphere(sc*[e1 e2 e3 e4 e5 e6 e7 e8], rs,'#D3D3D3',T*T1);

%% Collision detection
% Create list with all robot sphere centers:
  mk2_centers = [s3_centers; s4_centers; s60_centers; s61_centers; s62_centers];
  
% Check for collisions
  for i=1:size(stem_centers,1)
    for j=1:size(mk2_centers,1)
        a(i,j)=CheckCollision2(mk2_centers(j,:), stem_centers(i,:), rs);
    end
  end 
  
  hold off