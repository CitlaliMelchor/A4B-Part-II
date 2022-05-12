% ROBKIN: Compute the kinematic transformations
%         of a robot based on its kinematic parameters
%         specified by the matrix m see:
%         Introduction to Robotics Mechanics and Control,
%         J.J. Craig, 1986, Addison and Wesly.
%
%         function [Tt]=robkin(m);
%
%         Input:
%         m : matrix specifying the kinematic parameters of the robot
%             See Craig for the definition
%
%         Output:
%         Tt: [T01,T02, ... T0n] transformation matrices from
%             each link to the base (n = number of links)
%
%         Recursive relation: T0i=T0i-1*Ti-1i, i=1,2,..,n, T00=I
%         Ti-1i obtained from robot kinematic parameters
%
%         L.G. van Willigenburg 23-5-'96
%
  function [Tt]=robkin(m)

% Obtain the dimensions of input matrix m
% and initialize the transformation matrix T0i=T00=I
  [nl,ml]=size(m); T0i=eye(4);

% Initialize the matrix Tt which
% will collect all transformation matrices
  Tt=[];

% For each axis compute the transformation matrix T0i
% using the recursive relation mentioned above 
  for i=1:nl
    al=m(i,1); a=m(i,2); d=m(i,3); theta=m(i,4);
    ct=cos(theta); st=sin(theta); cal=cos(al); sal=sin(al);
    T0i=T0i*[  ct     -st     0     a   ;
             st*cal  ct*cal -sal -sal*d ;
             st*sal  ct*sal  cal  cal*d ;
               0       0      0     1  ];

% Collect T0i in Tt
    Tt=[Tt T0i];
  end
