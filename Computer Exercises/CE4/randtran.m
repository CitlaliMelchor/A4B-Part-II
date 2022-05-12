% RANDTRAN: Function generates a random transformation matrix
%
%          function [T]=randtran(pmax,seed);
%
%          Inputs, both optional:
%          pmax : Maximum absolute values position coordinates, default 1
%          seed : seed random number generator to reproduce results
%
%          Output:
%          T  :   Random Transformation matrix
%
%          L.G. van Willigenburg 15-5-'97, updated 12-6-2015
%
  function [T]=randtran(pmax,seed)

  if nargin<1; pmax=1; 
  elseif nargin>1; rand('seed',seed); end
  R=rpy(2*pi*rand,2*pi*rand,2*pi*rand); 
  P=2*pmax*(rand(3,1)-0.5); T=[R P; 0 0 0 1];
