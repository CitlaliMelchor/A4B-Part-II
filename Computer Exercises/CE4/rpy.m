% RPY: Function that computes the rotation matrix
%      given the role, pitch and yaw angles
%
%      [Rrpy]=rpy(g,b,a)
%
%      g,b,a:  role, pitch and yaw angle respectively
%      Rrpy :  rotation matrix
%
%      L.G. van Willigenburg 31-5-'96
%

 function [Rrpy]=rpy(g,b,a)

 ca=cos(a); sa=sin(a); cb=cos(b); sb=sin(b); cg=cos(g); sg=sin(g);

 Rrpy=[ca*cb ca*sb*sg-sa*cg ca*sb*cg+sa*sg;
       sa*cb sa*sb*sg+ca*cg sa*sb*cg-ca*sg;
         -sb       cb*sg           cb*cg  ];

