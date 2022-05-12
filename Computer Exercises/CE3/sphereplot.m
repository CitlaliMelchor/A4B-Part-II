% SPHEREPLOT: Function plots a sphere
%
%             sphereplot(ctr,rs,col,npl)
%
%             ctr : 3 or 4x1 vector with the first 3 components equal to
%                   the x,y and z coordinates of the center
%             rs  : radius of the sphere
%             col : FaceColor, e.g. 'r', [1,0,0],'#FF0000' (default blue)
%             npl : number of planes that approximate the sphere (default 10)
%
% GvW 10-6-2008
function sphereplot(ctr,rs,col,npl)

% Check inputs
if nargin<4; npl=10; end
if nargin<3; col='b'; end;
[n,m]=size(ctr); [nma]=max(n,m); [nmi]=min(n,m);
if nmi~=1 || (nma~=3 && nma~=4) error('ctr must be a vector of length 3 or 4'); end
[n,m]=size(rs); if n~=1 || m~=1; error('rs must be a scalar'); end

% Draw sphere
[xs,ys,zs]=sphere(npl); %cz=col*ones(size(zs)); % data of unit sphere
surf(rs*xs+ctr(1),rs*ys+ctr(2),rs*zs+ctr(3),'FaceColor',col,'FaceAlpha',0.3,'EdgeColor','none'); % Scale and plot approx. sphere
