% PATHGEN: Script file that generates a path for the MK2 robot
%          using the inverse kinematics and cubic spline interpolation
%
% 1. Explain how you are going to approximate the object by spheres.
% The robot and the objects in the environment are aproximated using the
% function beam2sphere.m, which transform the coordinates of a beam into
% spheres. The size of the spheres is fixed, and the number of spheres
% depends on the size of the beams. 

% 2. Explain how you can plan paths that collide and that do not collide.
% The two considered paths were planned using three positions which were
% created with the manmk2.m. For the collision path we made sure that one
% of the positions was inside the environment object. The paths were
% created using the spline function. 

% 3. Explain how you are going to use and adapt software from the previous 
% 3 assignments to perform the tasks mentioned under objective.
% beam2sphere remained the same and in mk2plot we added a loop to detect
% collisions between the object and the robot arm; the output of this
% function was changed so we retrieve a matrix of zeros and ones that
% indicate the spheres with collisions. We also used the pathgen script to
% check the two created paths, adding in the plotting loop a break function
% when collisions are detected as well as a warning that the selected path
% contains collision points. 

%% Load a robot path file, let the user decide if they want the collision
% path or the non-collision path
clear
clc
clf

prompt = 'Which robot path to use? \n 0 --> collision path\n 1 --> non-collision path\n (0/1) ';
x = input(string(prompt));
if x == 0
    load manmk2_collision; % collision path!
end
if x == 1
    load manmk2_no_collision; % no collision path!
end

P1 = pt(:,1);
P2 = pt(:,2);
P3 = pt(:,3);

% fixed parameter matrix
mf = [0,0,0; -pi/2,100,-130; 0,270,75; 0,230,55; pi/2,0,0];

% Uncomment below lines to get the thetas from the manmk2 path

% th1 = tht(:,1);
% th2 = tht(:,2);
% th3 = tht(:,3);

% specify the TCP transformation matrix for all 3 TCP positions and
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
a=[];
for i = 1:20
    T = robkin([mf,ths(:,i)]);
    a(:,:,i)= mk2plot(T,[120,10]);
    if ismember(1,a)
        break % Interrupt the process if collisions are found.
    end
    pause (0.1)
end

%% Warning if there is collision in this path
if ismember(1,a)
    warning('This path results in a collision of the robot with the environment!!!')
else
    disp('Path completed successfully!')
end

%% Transform the thetas from radians to encoder units used for the robot
% programm
thenc = rad2enc(ths);

% Create a mk2 robot file which can be used to control the mk2 robot
mk2prog(thenc);
