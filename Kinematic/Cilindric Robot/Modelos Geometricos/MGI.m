function q=MGI(x,y,z)

px = x;
py = y;
pz = z;
% q1 = atan2(-px,py);
q1 = atan2(-py,px);
R2 = pz;
R3 =(py*cos(q1))-(px*sin(q1));


q=[q1 R2 R3];
