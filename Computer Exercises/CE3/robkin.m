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
%             m = [ alpha, a, d, theta]
%
%         Output:
%         Tt: [T01,T02, ... T0n] transformation matrices from
%             each link to the base (n = number of links)
%
%         Recursive relation: T0i=T0i-1*Ti-1i, i=1,2,..,n, T00=I
%         Ti-1i obtained from robot kinematic parameters
%
%         T0(link) = T0(previous link)*Ti(link) <-
%
  function [Tt]=robkin(m)
 
% Obtain the dimensions of input matrix m
% and initialize the transformation matrix T0i=T00=I
  [nm,mm]=size(m); T0i=eye(4);
 
% Initialize the matrix Tt which
% will collect all transformation matrices
  Tt=[];
  
% For each axis compute the transformation matrix T0i
% using the recursive relation mentioned above 
  for i=1:nm
    T0i = T0i*...
         [cos(m(i,4)), -sin(m(i,4)), 0, m(i,2);...
          cos(m(i,1))*sin(m(i,4)), cos(m(i,1))*cos(m(i,4)), -sin(m(i,1)), -sin(m(i,1))*m(i,3);...
          sin(m(i,1))*sin(m(i,4)), sin(m(i,1))*cos(m(i,4)), cos(m(i,1)), cos(m(i,1))*m(i,3);...
          0, 0, 0, 1];        
    % Collect matrix T0i in Tt
    Tt=[Tt T0i];
  end
