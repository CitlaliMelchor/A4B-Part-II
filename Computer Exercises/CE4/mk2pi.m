% MK2PI : Adjust MK2 joint angles modulo 2*pi to ly inside their range
%
%        [th,vs]=mk2pi(th,df)
%
%        Inputs:
%        th  : matrix with joint angles each column containing one solution
%        df  : display flag, if non zero display out of range message
%
%        Outputs:
%        th  : matrix with joint angles modulo 2*pi
%        vs  : vs(j)=0 if column j has angles out of range, else 1
%
% GvW 18-6-2012
function [th,vs]=mk2pi(th,df)

% Check input
if nargin==1; df=0; end

% Maximum and minimum joint angles MK2 robot
% MK2 workspace A&B (page 2-6,2-7 ACL reference guide)
thmin=[-152.16 -156.58 -118.78 -113.12+90 -408.88]'*pi/180;
thmax=[ 154.30   38.19  113.02  113.48+90  388.84]'*pi/180;
thmin(3)=0; % Workspace A of the MK2

sz=size(th,2); vs=ones(sz,1);

for j=1:sz
% Adjust and check
  ih=th(:,j)>thmax; th(ih,j)=th(ih,j)-2*pi; 
  ih=th(:,j)<thmin; th(ih,j)=th(ih,j)+2*pi;
  if any([th(:,j)>thmax;th(:,j)<thmin]); 
    vs(j)=0;
    if df; 
      disp([' Column ',num2str(j),' of Mk2 joint angles out of range']);
      disp([thmax,th(:,j),thmin]);
    end
  end
end