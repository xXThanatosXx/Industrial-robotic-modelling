function q = mgi_Polar(x1,x2,x3)
Px= x1;
Py= x2;
Pz=x3;


D1 =1;
   
% Despejando
q1 = atan2(Py,Px);
% raiz positiva
% q2 = atan2(sqrt(Px^2+Py^2),D1-Pz);
% raiz negativa
q2 = atan2(-sqrt(Px^2+Py^2),Pz-D1);
q3 = (cos(q2)*(Pz-D1))-(sin(q2)*(sqrt(Px^2+Py^2)));
   
q = [q1 q2 q3];

