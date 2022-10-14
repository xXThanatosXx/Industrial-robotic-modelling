function q=MGI(x,y,z)

px = x;
py = y;
pz = z;

% Para q1
% q1 = Px*cos(teta)+Py*sin(teta)
% Del compendio de formulas se obtiene que: 
% A*Cos(teta)+B*sin(teta) se tiene dos soluciones
% primera solución cuando atan2(A,-B):
% q1 =atan2(px,-py);
% Segunda solución cuando atan2(A,-B):
q1 =atan2(-px,py);



R2 = pz;
R3 =(py*cos(q1))-(px*sin(q1));
q=[q1 R2 R3];
