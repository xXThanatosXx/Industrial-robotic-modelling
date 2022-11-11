% Inverse Dynamic Model using Newton-Euler Algorithm
% Robot with rigid joints and fixed base
% 
% 
% Geometric parameters
% j       ant     sigma   mu      gamma   b       alpha   d       theta   r       
% 1       0       0       1       0       0       0       0       th1     0       
% 2       1       1       1       0       0       0       0       0       R2      
% 3       2       1       1       0       0       -pi/2   0       0       R3      
% 4       3       0       1       0       0       0       0       th4     0       
% 
% Dynamic inertia parameters
% j       XX      XY      XZ      YY      YZ      ZZ      MX      MY      MZ      M       IA      
% 1       XX1     XY1     XZ1     YY1     YZ1     ZZ1     MX1     MY1     MZ1     M1      IA1     
% 2       XX2     XY2     XZ2     YY2     YZ2     ZZ2     MX2     MY2     MZ2     M2      IA2     
% 3       XX3     XY3     XZ3     YY3     YZ3     ZZ3     MX3     MY3     MZ3     M3      IA3     
% 4       XX4     XY4     XZ4     YY4     YZ4     ZZ4     MX4     MY4     MZ4     M4      IA4     
% 
% External forces and joint parameters
% j       FX      FY      FZ      CX      CY      CZ      FS      FV      QP      QDP     GAM     eta     k       
% 1       0       0       0       0       0       0       FS1     FV1     QP1     QDP1    GAM1    0       0       
% 2       0       0       0       0       0       0       FS2     FV2     QP2     QDP2    GAM2    0       0       
% 3       0       0       0       0       0       0       FS3     FV3     QP3     QDP3    GAM3    0       0       
% 4       0       0       0       0       0       0       FS4     FV4     QP4     QDP4    GAM4    0       0       
% 
% Base velicities parameters
% axis    W0      WP0     V0      VP0     G       
% X       0       0       0       0       0       
% Y       0       0       0       0       0       
% Z       0       0       0       0       G3      



function GAM = MDI(th1,th2,th3,th4,QP1,QP2,QP3,QP4,QDP1,QDP2,QDP3,QDP4)


C1 = cos(th1);
S1 = sin(th1);
C4 = cos(th4);
S4 = sin(th4);

% Gravedad
G3 = -9.81;
% Distancias
R3 = 0.45;
R2 = 0.45;
% Masas de las articulaciones
M1 = 0.45;
M2 = 0.45;
M3 = 0.45;
M4 = 0.45;  

% tabla de parametros del robot 
XX1 = 0;
XX2 = 0;
XX3 = 0; 
XX4 = 0;

XY1 = 0;
XY2 = 0;
XY3 = 0;
XY4 = 0;


XZ1 = 0;
XZ2 = 0;
XZ3 = 0;
XZ4 = 0;
YZ1 = 0;
YZ2 = 0;
YZ3 = 0;
YZ4 = 0;
YY1 = 0;
YY2 = 0;
YY3 = 0;
YY4 = 0;
ZZ1 = 0;
ZZ2 = 0;
ZZ3 = 0;
ZZ4 = 0;

% Momentos de inercia
MX1 = 0.001;
MX2 = 0.001;
MX3 = 0.001;
MX4 = 0.001;

MY1 = 0.001;
MY2 = 0.001;
MY3 = 0.001;
MY4 = 0.001;

MZ1 = 0.001;
MZ2 = 0.001;
MZ3 = 0.001;
MZ4 = 0.001;

IA1 = 0.01;
IA2 = 0.01;
IA3 = 0.01;
IA4 = 0.01;

% fuerzas externas
FS1=0.0;
FS2=0.0;
FS3=0.0;
FS4=0.0;
	
FV1=0.0;
FV2=0.0;
FV3=0.0;
FV4=0.0;	


% MDI Symoro
DV61 = QP1^2;
VP32 = -G3 + QDP2;
VSP13 = -QDP1*R3;
VSP23 = -DV61*R3;
VP13 = -2*QP1*QP3 + VSP13;
VP33 = QDP3 + VSP23;
W14 = -QP1*S4;
W24 = -C4*QP1;
WP14 = -QDP1*S4 + QP4*W24;
WP24 = -C4*QDP1 - QP4*W14;
DV14 = W14^2;
DV24 = W14*W24;
DV34 = QP4*W14;
DV44 = W24^2;
DV54 = QP4*W24;
DV64 = QP4^2;
U114 = -DV44 - DV64;
U214 = DV24 + QDP4;
U314 = DV34 - WP24;
U124 = DV24 - QDP4;
U224 = -DV14 - DV64;
U324 = DV54 + WP14;
U134 = DV34 + WP24;
U234 = DV54 - WP14;
U334 = -DV14 - DV44;
VP14 = C4*VP13 - S4*VP32;
VP24 = -C4*VP32 - S4*VP13;
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
F14 = M4*VP14 + MX4*U114 + MY4*U124 + MZ4*U134;
F24 = M4*VP24 + MX4*U214 + MY4*U224 + MZ4*U234;
F34 = M4*VP33 + MX4*U314 + MY4*U324 + MZ4*U334;
PSI14 = QP4*XZ4 + W14*XX4 + W24*XY4;
PSI24 = QP4*YZ4 + W14*XY4 + W24*YY4;
PSI34 = QP4*ZZ4 + W14*XZ4 + W24*YZ4;
No14 = -PSI24*QP4 + PSI34*W24 + QDP4*XZ4 + WP14*XX4 + WP24*XY4;
No24 = PSI14*QP4 - PSI34*W14 + QDP4*YZ4 + WP14*XY4 + WP24*YY4;
No34 = -PSI14*W24 + PSI24*W14 + QDP4*ZZ4 + WP14*XZ4 + WP24*YZ4;
N14 = MY4*VP33 - MZ4*VP24 + No14;
N24 = -MX4*VP33 + MZ4*VP14 + No24;
N34 = MX4*VP24 - MY4*VP14 + No34;
FDI14 = C4*F14 - F24*S4;
FDI24 = C4*F24 + F14*S4;
E13 = F13 + FDI14;
E23 = F23 + FDI24;
E33 = F33 + F34;
N13 = C4*N14 + MY3*VP33 + MZ3*VP32 - N24*S4 + No13;
N23 = C4*N24 - MX3*VP33 + MZ3*VP13 + N14*S4 + No23;
N33 = -MX3*VP32 - MY3*VP13 + N34 + No33;
E12 = E13 + F12;
E22 = E33 + F22;
E32 = -E23 + F32;
N12 = -E23*R3 + MY2*VP32 + N13 + No12;
N22 = -MX2*VP32 + N33 + No22;
N32 = -E13*R3 - N23 + No32;
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
GAM3 = E33 + FS3*sign(QP3) + FV3*QP3 + IA3*QDP3;
GAM4 = FS4*sign(QP4) + FV4*QP4 + IA4*QDP4 + N34;

% Fin SYmoro

GAM(1) = GAM1;
GAM(2) = GAM2;
GAM(3) = GAM3;
GAM(4) = GAM4;
