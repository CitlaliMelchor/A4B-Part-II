function beam2sphere(beam, rs, col, T)
% This function transform the coordinates of a beam into spheres
% Input:    
%           beam - e1 e2 e3 e4 e5 e6 e7 e8, coordinates of the edges. 
%           rs   - radius of the spheres, mm
%           col  - FaceColor, e.g. 'r', [1,0,0],'#FF0000'
%           T    - Transformation matrix
% Output: 
%           plot of the spheres

% Order the edges from bottom to top in clockwise order.
beam=sortrows(beam',[3,2])';

% Stores the coordinate of the necessary edges
e1=beam(:,1)';
e2=beam(:,2)';
e3=beam(:,3)';
e5=beam(:,5)';

% Define the beam length, depth and height
L = e2(1)-e1(1); if L>0; L=ceil(L);else L=floor(L); end
D = e3(2)-e1(2); if D>0; D=ceil(D);else D=floor(D); end
H = e5(3)-e1(3); if H>0; H=ceil(H);else H=floor(H); end

% Side of the cube
r = 2*rs/sqrt(3); 

% Number cubes in each direction 
nL = L/r; 
nD = D/r; 
nH = H/r;

for i=1:ceil(abs(nL))
    for j=1:ceil(abs(nD))
        for k=1:ceil(abs(nH))
            center = e1+[((i-1))*(L/abs(nL)),...
                         ((j-1))*(D/abs(nD)),...
                         ((k-1))*(H/abs(nH))];
            center = T*[center, 1]';
            sphereplot(center(1:4,:),rs,col,10)
             hold on
        end
    end
end
end

