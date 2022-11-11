function q=mgi(x,y,z)

% A1 = y, S1 =x, P3 = z

px = x;
py = y;
pz = z;

% Equations:
% # Solving type 1
R2 = pz;
% # Solving type 3
% # X1*sin(th1) + Y1*cos(th1) = Z1
% # X2*sin(th1) + Y2*cos(th1) = Z2

q1 = atan2(-px, py);
% R2=px*sin(q1) + py*cos(q1);
% # Solving type 3
% # X1*sin(th4) + Y1*cos(th4) = Z1
% # X2*sin(th4) + Y2*cos(th4) = Z2
q4 = atan2(px,py);

R3=px*sin(q4) + py*cos(q4);

q=[q1 q4 R2 R3];
