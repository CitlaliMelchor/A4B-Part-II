function [a]=CheckCollision2(center1, center2, rs)
% A function to check the collision between two surfaces using the
% intersection of their surfaces
% Made by: Melchor Ramirez, Citlali (citlali.melchorramirez@wur.nl) 
%          Westeringh van de, Nick (nick.vandewesteringh@wur.nl)
% Input:
%          center1, center2 : x, y, z coordinates of the centers of the
%                             spheres. 
%          rs               : radius of the spheres 
% Output:
%           a : 1 if collision exist, 
%               0 if there is no collision. 

% Smallest euclidean distance between the surfaces of the two spheres. 
  distance = norm(center1-center2)-2*rs;

  if distance > 0
      a = 0;
  else
      disp('collision detected')
      sphereplot(center1,rs,'r');
      sphereplot(center2,rs,'r');
      a = 1;
  end
