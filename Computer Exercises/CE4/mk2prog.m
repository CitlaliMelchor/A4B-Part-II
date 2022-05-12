% MK2PROG: Function generates an MK2 control program in file MK2.DNL
%          The Program moves the MK2 robot along the points
%          specified by thetat, which must be in ENCODERCOUNTS.
%          The movement specified by thetat is executed using the
%          SPLINE instruction of the MK2 robot.
%
%          mk2prog(thenct)
%
%  Input:
%  thenct: [thenc1; thenc2; thenc3; thenc4; thenc5]
%           matrix which contains the encodercounts,
%           first row  : encodercounts of successive values of theta1,
%           second row : encodercounts of successive values of theta2 etc.
%
%          Output: File MK2.DNL which contains the MK2 program code an
%          example of which is given below
%
%          # Part 1: definition of positions in encodercounts
%          SETPV P[1]
%           -58612
%           -10929
%            58771
%            72104
%           -47900
% 
%          SETPV P[2]
%           -56132
%           -12816
%            56478
%            70567
%           -45754
%
%          ...
%
%          SETPV P[42]
%            43055
%           -88263
%           -35229
%             9104
%            40100
%
%          # Part 2: program definitions
%          PROGRAM MK2
%            SPEED 20
%            MOVE P[1]
%            SPEED 50
%            MOVES P 1 42
%          END
%
% GvW; updated 1-8-2016
function mk2prog(thenct)

if nargin~=1; error('One input argument required'); end;
[n1,n]=size(thenct);
if n1~=5; error('Input argument must have 5 rows'); end;

fid = fopen('mk2.dnl','w');
str='# Part 1: declaration and definition of positions in encodercounts\n';
fprintf(fid,str);
for i=1:n
  str=['\nSETPV P[' num2str(i) ']\n']; fprintf(fid,str);
  for k=1:5; fprintf(fid,'%8.0f\n',thenct(k,i)); end;
end;
str='\n# Part 2: program definitions\n'; fprintf(fid,str);
str=['PROGRAM MK2\n  MOVE P[1]\n  MOVES P 1 ' num2str(n) ' 1000\nEND'];
fprintf(fid,str);
fclose(fid);