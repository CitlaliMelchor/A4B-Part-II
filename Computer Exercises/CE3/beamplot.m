% BEAMPLOT: Function plots a beam
%
%          function beamplot(beam,c)
%
%          beam : matrix containing 8 columnvectors which specify
%                 the corners in clock or anticlockwise order for the
%                 bottom 4 and top 4 and such that the bottom 4
%                 allign with the top 4.
%          c    : color and edge/surface string (optional)
%                 examples 'r' (red edges) 'gs' (green surface)
%                 default 'r'
%
%          Example: plot a cube
%
%          figure(gcf); clf; r1=1; l=2*r1;
%          axis([-l l -l l -l l]); hold on;
%          e1=[-r1;  r1; r1]; e2=[-r1; -r1; r1];
%          e3=[-r1; -r1;-r1]; e4=[-r1;  r1;-r1]; % bottom 4
%          e5=[ r1;  r1; r1]; e6=[ r1; -r1; r1];
%          e7=[ r1; -r1;-r1]; e8=[ r1;  r1;-r1]; % top 4
%          beamplot([e1 e2 e3 e4 e5 e6 e7 e8]); hold off; figure(gcf);
%
%          L.G. van Willigenburg 23-5-'96
%
  function beamplot(beam,c)

  [n,m]=size(beam);
  if n<3 | m~=8; error('The first input argument should be a 3x8 or 4x8 matrix'); end;

  if nargin<2; c='r'; end;
  if isempty(c); c='r'; end;

  [nc,mc]=size(c); if mc>1; c=c(1); end;

% If color edge/surface switch is not a string
  if ~isstr(c)
    if c<0.33; c='r';
    elseif c<0.66; c='g';
    else c='b'; end;
  end;

  if mc<2;
% Plot edges
    k=beam(:,[1:4 1]); plot3(k(1,:),k(2,:),k(3,:),c);
    k=beam(:,[5:8 5]); plot3(k(1,:),k(2,:),k(3,:),c);
    k=beam(:,[1 5]); plot3(k(1,:),k(2,:),k(3,:),c);
    k=beam(:,[2 6]); plot3(k(1,:),k(2,:),k(3,:),c);
    k=beam(:,[3 7]); plot3(k(1,:),k(2,:),k(3,:),c);
    k=beam(:,[4 8]); plot3(k(1,:),k(2,:),k(3,:),c);
  else
% Plot surfaces
    k=beam(:,[1:4 1]); fill3(k(1,:),k(2,:),k(3,:),c);
    k=beam(:,[5:8 5]); fill3(k(1,:),k(2,:),k(3,:),c);
    k=beam(:,[1 2 6 5]); fill3(k(1,:),k(2,:),k(3,:),c);
    k=beam(:,[4 3 7 8]); fill3(k(1,:),k(2,:),k(3,:),c);
    k=beam(:,[4 1 5 8]); fill3(k(1,:),k(2,:),k(3,:),c);
    k=beam(:,[2 3 7 6]); fill3(k(1,:),k(2,:),k(3,:),c);
  end
