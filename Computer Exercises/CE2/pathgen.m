% PATHGEN: Script file that generates a path for the MK2 robot
%          using the inverse kinematics and cubic spline interpolation
%
% The function manmk2 plots and allows you to manually move each axis
% of the MK2 robot on the screen. It is also capable of storing positions,
% transformation matrices and joint angles of the MK2. This function will
% be very helpful when performing this exercise.

% Choose (arbitrarily) the Cartesian coordinates [px;py;pz], expressed in
% the universal frame 0, of 3 points within the MK2 workspace but outside
% "the L1 circle" (see sheets ?? of lecture 2). One is the starting point,
% one an intermediate point and one an end point of a robot motion.
% Specify these 3 points by means of 3 column vectors [px;py;pz]
% expressed in the universal MK2 frame (frame 0). The Z coordinate pz of
% all 3 points should be above -250 (mm.) to prevent collision
% with the table on which the robot is mounted.

%% 
% Some of the solutions are not possible to perform because there is a
% distance between the links, thus the points are constrainted to the robot
% workspace. This can be checked with forward kinematics and when calling
% the function invkmk2. The values for calculating some of the angles are
% not real when there is no solution. 


%%
% Load the 3 by manmk2.m created robot positions.
load manmk2;
P1 = pt(:,1);
P2 = pt(:,2);
P3 = pt(:,3);

% Fixed parameter matrix
mf = [0,0,0; -pi/2,100,-130; 0,270,75; 0,230,55; pi/2,0,0];

% Uncomment below lines to get the thetas from the manmk2 path

% th1 = tht(:,1);
% th2 = tht(:,2);
% th3 = tht(:,3);

% Specify the TCP transformation matrix for all 3 TCP positions and
% orientations.
Tt1 = Tt(:,1:4);
Tt2 = Tt(:,5:8);
Tt3 = Tt(:,9:12);

% Using inverse kinematics to get the thetas from the desired robot
% positions
th1 = invkmk2(Tt1, mf);
th2 = invkmk2(Tt2, mf);
th3 = invkmk2(Tt3, mf);
Tm1 = robkin([mf,th1]);
Tm2 = robkin([mf,th2]);
Tm3 = robkin([mf,th3]);

% Uncomment lines below to plot the three desired robot postions

% figure(1)
% mk2plot(Tm1,[15,45]);
% figure(2)
% mk2plot(Tm2,[15,45]);
% figure(3)
% mk2plot(Tm3,[15,45]);

% Test if the three positions of the robot can be reached
[th,vs]=mk2pi([th1,th2,th3]);
% vs values are all 1 --> all positions are reachable.

% Use cubic spline to create intermediate values between the three robot
% positions
ths=spline([1,10,20],th,1:20);

%% Create figure and plot the path of the robot
figure(4);
for i = 1:20
    T = robkin([mf,ths(:,i)]);
    mk2plot(T,[15,45]);
    pause
end

%% Transform the thetas from radians to encoder units used for the robot
% programm
thenc = rad2enc(ths);

% Create a mk2 robot file which can be used to control the mk2 robot
mk2prog(thenc);
