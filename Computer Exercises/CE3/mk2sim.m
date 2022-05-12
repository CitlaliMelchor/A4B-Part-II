% MK2SIM: Script-file that plots the MK2 robot or silumates a robot motion
% with spheres approximation. 
% 
% The beams are approximated by spheres of the same size and the number of
% spheres per beam depends on the dimensions of it. 
%
%         See Introduction to Robotics Mechanics and Control,
%         J.J. Craig, 1989, Addison and Wesly, pp. 93-98, 136-141.
%         Observe the error on pp.98.
%
%
%% Specify the FIXED robot parameters
% Angles of the links
alpha = [0, -pi/2, 0, 0, pi/2];
% Distance along x axis
a = [0, 100, 270, 230, 0];
% d = distance along z axis
d = [0, -130, 75, 55, 0];

%% Specify the VARIABLE robot parameters   
% Minimum and maximum value of each joint angle
thmin=[-152.16 -156.58 -118.78 -113.12+90 -90]; 
thmax=[ 154.30   38.19  113.02  113.48+90  90]; 

nstep = 20; % Number of intermidiate steps

% Construct the arrays with the angles per link
TH1 = deg2rad(thmin(1):(thmax(1)-thmin(1))/nstep:thmax(1)); %L2
TH2 = deg2rad(thmin(2):(thmax(2)-thmin(2))/nstep:thmax(2)); %L3 
TH3 = deg2rad(thmin(3):(thmax(3)-thmin(3))/nstep:thmax(3)); %L4
TH4 = deg2rad(thmin(4):(thmax(4)-thmin(4))/nstep:thmax(4)); %L5
TH5 = deg2rad(thmin(5):(thmax(5)-thmin(5))/nstep:thmax(5)); %L6

%% Specify the MATRIX of robot parameters
% Matrix m containts the fixed and variable parameters of the robot, for
% each time step i; size 5x4x11. 
for i=1:nstep+1
    m(:,:,i)=[alpha', a', d' , [TH1(i),TH2(i),TH3(i),TH4(i),TH5(i)]']; 
end 

%% Compute the TRANSFORMATION MATRICES and PLOT the result
% Initialize the figure
fg = figure(1);
set(fg, 'units','normalized', 'outerposition', [0 0.05 1 0.95]); 

for i=1:nstep+1
    % Calculate the transformation matrix for time step i
    Tt = robkin(m(:,:,i));
    % Apply the transformation matrix to the plot of the robot
    mk2plot(Tt,[-50,30]) 
    drawnow;
    pause%(0.01);
end
