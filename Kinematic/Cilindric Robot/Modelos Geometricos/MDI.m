% Inverse Dynamic Model using Newton-Euler Algorithm
% Robot with rigid joints and fixed base
% 
% 
% Geometric parameters
% j       ant     sigma   mu      gamma   b       alpha   d       theta   r       
% 1       0       0       1       0       0       0       0       th1     0       
% 2       1       1       1       0       0       0       0       0       R2      
% 3       2       1       1       0       0       -pi/2   0       0       R3      
% 
% Dynamic inertia parameters
% j       XX      XY      XZ      YY      YZ      ZZ      MX      MY      MZ      M       IA      
% 1       XX1     XY1     XZ1     YY1     YZ1     ZZ1     MX1     MY1     MZ1     M1      IA1     
% 2       XX2     XY2     XZ2     YY2     YZ2     ZZ2     MX2     MY2     MZ2     M2      IA2     
% 3       XX3     XY3     XZ3     YY3     YZ3     ZZ3     MX3     MY3     MZ3     M3      IA3     
% 
% External forces and joint parameters
% j       FX      FY      FZ      CX      CY      CZ      FS      FV      QP      QDP     GAM     eta     k       
% 1       0       0       0       0       0       0       FS1     FV1     QP1     QDP1    GAM1    0       0       
% 2       0       0       0       0       0       0       FS2     FV2     QP2     QDP2    GAM2    0       0       
% 3       0       0       0       0       0       0       FS3     FV3     QP3     QDP3    GAM3    0       0       
% 
% Base velicities parameters
% axis    W0      WP0     V0      VP0     G       
% X       0       0       0       0       0       
% Y       0       0       0       0       0       
% Z       0       0       0       0       G3     


function GAM = MDI(th1,QP1,QP2,QP3,QDP1,QDP2,QDP3)

% Definición de coseno y seno del angulo theta 1 c1=cos(cos(q1)),s1=(sin(q1))
C1 = cos(th1);
S1 = sin(th1);

% Gravedad
G3 = -9.81;
% Distancias 
R2 = 0.45;
R3 = 0.45;

% Masa de las articulaciones J
M1= 0.45;
M2= 0.85;
M3= 0.45;


% Parametros del robot
XX1 = 0;
XX2 = 0;
XX3 = 0; 
XY1 = 0;
XY2 = 0;
XY3 = 0;
XZ1 = 0;
XZ2 = 0;
XZ3 = 0;
YZ1 = 0;
YZ2 = 0;
YZ3 = 0;
YY1 = 0;
YY2 = 0;
YY3 = 0;
ZZ1 = 0;
ZZ2 = 0;
ZZ3 = 0;
% Primeros momentos de inercia de las articulaciones
% con respecto a los ejers(X,Y,Z)
MX1 = 0.001;
MX2 = 0.001;
MX3 = 0.001;

MY1 = 0.001;
MY2 = 0.001;
MY3 = 0.001;

MZ1 = 0.001;
MZ2 = 0.001;
MZ3 = 0.001;
% Parámetrso inerciasles de las articulaciones J
IA1 = 0.04;
IA2 = 0.04;
IA3 = 0.04;

% Fuerzas de rozamiento
FS1=0.0;
FS2=0.0;
FS3=0.0;
	
FV1=0.0;
FV2=0.0;
FV3=0.0;


% MDI Symoro
C1 = cos(th1);
S1 = sin(th1);

DV61 = QP1^2;
VP32 = -G3 + QDP2;
VSP13 = -QDP1*R3;
VSP23 = -DV61*R3;
VP13 = -2*QP1*QP3 + VSP13;
VP33 = QDP3 + VSP23;
F11 = -DV61*MX1 - MY1*QDP1;
F21 = -DV61*MY1 + MX1*QDP1;
F31 = -G3*M1;
PSI11 = QP1*XZ1;
PSI21 = QP1*YZ1;
PSI31 = QP1*ZZ1;
No11 = -PSI21*QP1 + QDP1*XZ1;
No21 = PSI11*QP1 + QDP1*YZ1;
No31 = QDP1*ZZ1;
F12 = -DV61*MX2 - MY2*QDP1;
F22 = -DV61*MY2 + MX2*QDP1;
F32 = M2*VP32;
PSI12 = QP1*XZ2;
PSI22 = QP1*YZ2;
PSI32 = QP1*ZZ2;
No12 = -PSI22*QP1 + QDP1*XZ2;
No22 = PSI12*QP1 + QDP1*YZ2;
No32 = QDP1*ZZ2;
F13 = -DV61*MX3 + M3*VP13 - MZ3*QDP1;
F23 = -M3*VP32;
F33 = -DV61*MZ3 + M3*VP33 + MX3*QDP1;
PSI13 = -QP1*XY3;
PSI23 = -QP1*YY3;
PSI33 = -QP1*YZ3;
No13 = -PSI33*QP1 - QDP1*XY3;
No23 = -QDP1*YY3;
No33 = PSI13*QP1 - QDP1*YZ3;
N13 = MY3*VP33 + MZ3*VP32 + No13;
N23 = -MX3*VP33 + MZ3*VP13 + No23;
N33 = -MX3*VP32 - MY3*VP13 + No33;
E12 = F12 + F13;
E22 = F22 + F33;
E32 = -F23 + F32;
N12 = -F23*R3 + MY2*VP32 + N13 + No12;
N22 = -MX2*VP32 + N33 + No22;
N32 = -F13*R3 - N23 + No32;
E11 = E12 + F11;
E21 = E22 + F21;
E31 = E32 + F31;
N11 = -E22*R2 - G3*MY1 + N12 + No11;
N21 = E12*R2 + G3*MX1 + N22 + No21;
N31 = N32 + No31;
FDI11 = C1*E11 - E21*S1;
FDI21 = C1*E21 + E11*S1;

GAM1 = FS1*sign(QP1) + FV1*QP1 + IA1*QDP1 + N31;
GAM2 = E32 + FS2*sign(QP2) + FV2*QP2 + IA2*QDP2;
GAM3 = F33 + FS3*sign(QP3) + FV3*QP3 + IA3*QDP3;


% Fin SYmoro

GAM(1) = GAM1;
GAM(2) = GAM2;
GAM(3) = GAM3;

end