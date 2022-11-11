function pos=mgd(q1,q4,R2,R3)


C1 = cos(q1);
S1 = sin(q1);
T0T314 = -R3*S1;
T0T324 = C1*R3;
C4 = cos(q4);
S4 = sin(q4);
T0T411 = C1*C4;
T0T421 = C4*S1;
T0T431 = -S4;
T0T412 = -C1*S4;
T0T422 = -S1*S4;
T0T432 = -C4;
T0T413 = -S1;
T0T423 = C1;
T0T433 = 0;
T0T414 = T0T314;
T0T424 = T0T324;
T0T434 = R2;


% Matriz OT4
U0=[ T0T411,  T0T412,   T0T413, T0T414;
     T0T421,  T0T422,   T0T423, T0T424;
     T0T431,  T0T432,   T0T433, T0T434;
     0,   0,   0,      1];

xa = U0(1,4);
ya = U0(2,4);
za = U0(3,4);


pos = [xa ya za];