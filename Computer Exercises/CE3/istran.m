% ISTRAN: Function that checks wether T is a transformation matrix
%         using the special structure of its inverse
%
%         function [chk]=istran(T);
%
%         T  :   Transformation matrix to be checked
%         chk:   1 if T is a transformation matrix, 0 if not.
%
%         L.G. van Willigenburg 31-5-'96
%
%         if R = T(1:3,1:3); P = T(1:3,4); then
%         inv(T) = [R' -R'*P; 0 0 0 1] must hold
%         where R and P are the rotation and
%         translation matrix inside T.
%

  function [chk]=istran(T);

  [n,m]=size(T);

  if n~=4 | m~=4
    chk=0;
  else
    R=T(1:3,1:3); P=T(1:3,4);
    T1=[R' -R'*P; 0 0 0 1]; T1=T*T1;
    if max(max(abs(T1-eye(4))))>1e-3
      chk=0;
    else
      chk=1;
    end;
  end;  
