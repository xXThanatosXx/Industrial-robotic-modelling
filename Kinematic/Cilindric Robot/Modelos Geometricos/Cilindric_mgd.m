function pos=Cilindric_mgd(q1,R2,R3)

U0 = [cos(q1), 0, -sin(q1), -R3*sin(q1); 
      sin(q1), 0,  cos(q1), R3*cos(q1); 
            0 -1,        0,         R2; 
       0, 0, 0, 1];

x = U0(1,4);
y = U0(2,4);
z = U0(3,4);

pos = [x y z];
