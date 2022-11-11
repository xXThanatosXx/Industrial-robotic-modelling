% Direct Dynamic Model using Newton-Euler Algorithm
% Robot with fixed base
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


function [sys, x0,str,ts] = DDM(t,x,u,flag,QI)
%   See csfunc.m for a continuous linear state space example   
%   See sfuntmpl.m for a general S-function template.
%

switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes(QI);

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,

   sys=mdlDerivatives(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u);

  %%%%%%%%%%%%%%%%%%%
  % Unhandled flags %
  %%%%%%%%%%%%%%%%%%%
  case { 2, 4, 9 },
    sys = [];

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    error(['Unhandled flag = ',num2str(flag)]);

end


%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts]=mdlInitializeSizes(QI)

sizes = simsizes;
sizes.NumContStates  = 8;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 8;
sizes.NumInputs      = 4;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);

%
% initialize the initial conditions

x0=[QI(1);QI(2);QI(3);QI(4);0;0;0;0];

%
% str is always an empty matrix
%
str = [];

%
% initialize the array of sample times
%
ts  = [0 0];

% end mdlInitializeSizes
%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%

%function sys=mdlDerivatives(t,x,u,L1,L2,ZZ1,ZZ2,MX2,MY2,FV1,FV2,FS1,FS2)
function sys=mdlDerivatives(t,x,u)
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

% torques
GAM1=u(1);
GAM2=u(2);
GAM3=u(3);
GAM4=u(4);

C1 = cos(x(1));
S1 = sin(x(1));


C4 = cos(x(4));
S4 = sin(x(4));


% velocidades
QP1=x(5);
QP2=x(6);
QP3=x(7);
QP4=x(8);


	
% Code symoro%%
JW11 = QP1*XZ1;
JW21 = QP1*YZ1;
JW31 = QP1*ZZ1;
KW11 = -JW21*QP1;
KW21 = JW11*QP1;
SW11 = -MX1*QP1^2;
SW21 = -MY1*QP1^2;
JW12 = QP1*XZ2;
JW22 = QP1*YZ2;
JW32 = QP1*ZZ2;
KW12 = -JW22*QP1;
KW22 = JW12*QP1;
SW12 = -MX2*QP1^2;
SW22 = -MY2*QP1^2;
WQ13 = -QP1*QP3;
LW13 = 2*WQ13;
LW33 = -QP1^2*R3;
JW13 = -QP1*XY3;
JW23 = -QP1*YY3;
JW33 = -QP1*YZ3;
KW13 = -JW33*QP1;
KW33 = JW13*QP1;
SW13 = -MX3*QP1^2;
SW33 = -MZ3*QP1^2;
WI14 = -QP1*S4;
WI24 = -C4*QP1;
WQ14 = QP4*WI24;
WQ24 = -QP4*WI14;
JW14 = QP4*XZ4 + WI14*XX4 + WI24*XY4;
JW24 = QP4*YZ4 + WI14*XY4 + WI24*YY4;
JW34 = QP4*ZZ4 + WI14*XZ4 + WI24*YZ4;
KW14 = -JW24*QP4 + JW34*WI24;
KW24 = JW14*QP4 - JW34*WI14;
KW34 = -JW14*WI24 + JW24*WI14;
SW14 = -QP4*(MX4*QP4 - MZ4*WI14) + WI24*(-MX4*WI24 + MY4*WI14);
SW24 = QP4*(-MY4*QP4 + MZ4*WI24) - WI14*(-MX4*WI24 + MY4*WI14);
SW34 = WI14*(MX4*QP4 - MZ4*WI14) - WI24*(-MY4*QP4 + MZ4*WI24);
GW4 = -FS4*sign(QP4) - FV4*QP4 + GAM4 - KW34;
JD4 = 1/(IA4 + ZZ4);
JU14 = -JD4*MY4;
JU24 = JD4*MX4;
JU44 = JD4*XZ4;
JU54 = JD4*YZ4;
JU64 = JD4*ZZ4;
GK114 = JU14*MY4 + M4;
GK214 = JU24*MY4;
GK414 = JU44*MY4;
GK514 = JU54*MY4 + MZ4;
GK614 = JU64*MY4 - MY4;
GK124 = -JU14*MX4;
GK224 = -JU24*MX4 + M4;
GK424 = -JU44*MX4 - MZ4;
GK524 = -JU54*MX4;
GK624 = -JU64*MX4 + MX4;
GK144 = -JU14*XZ4;
GK244 = -JU24*XZ4 - MZ4;
GK444 = -JU44*XZ4 + XX4;
GK544 = -JU54*XZ4 + XY4;
GK644 = -JU64*XZ4 + XZ4;
GK154 = -JU14*YZ4 + MZ4;
GK254 = -JU24*YZ4;
GK454 = -JU44*YZ4 + XY4;
GK554 = -JU54*YZ4 + YY4;
GK654 = -JU64*YZ4 + YZ4;
GK164 = -JU14*ZZ4 - MY4;
GK264 = -JU24*ZZ4 + MX4;
GK464 = -JU44*ZZ4 + XZ4;
GK564 = -JU54*ZZ4 + YZ4;
GK664 = -JU64*ZZ4 + ZZ4;
NG14 = GK144*WQ14 + GK154*WQ24;
NG24 = GK244*WQ14 + GK254*WQ24;
NG34 = -MX4*WQ24 + MY4*WQ14;
NG44 = GK444*WQ14 + GK454*WQ24;
NG54 = GK544*WQ14 + GK554*WQ24;
NG64 = GK644*WQ14 + GK654*WQ24;
VS14 = GW4*JU14 + NG14;
VS24 = GW4*JU24 + NG24;
VS44 = GW4*JU44 + NG44;
VS54 = GW4*JU54 + NG54;
VS64 = GW4*JU64 + NG64;
AP14 = SW14 + VS14;
AP24 = SW24 + VS24;
AP34 = NG34 + SW34;
AP44 = KW14 + VS44;
AP54 = KW24 + VS54;
AP64 = KW34 + VS64;
GX114 = C4*GK114 - GK214*S4;
GX214 = C4*GK214 + GK114*S4;
GX414 = C4*GK414 - GK514*S4;
GX514 = C4*GK514 + GK414*S4;
GX124 = C4*GK124 - GK224*S4;
GX224 = C4*GK224 + GK124*S4;
GX424 = C4*GK424 - GK524*S4;
GX524 = C4*GK524 + GK424*S4;
GX434 = C4*MY4 + MX4*S4;
GX534 = -C4*MX4 + MY4*S4;
GX144 = C4*GK144 - GK244*S4;
GX244 = C4*GK244 + GK144*S4;
GX444 = C4*GK444 - GK544*S4;
GX544 = C4*GK544 + GK444*S4;
GX154 = C4*GK154 - GK254*S4;
GX254 = C4*GK254 + GK154*S4;
GX454 = C4*GK454 - GK554*S4;
GX554 = C4*GK554 + GK454*S4;
GX164 = C4*GK164 - GK264*S4;
GX264 = C4*GK264 + GK164*S4;
GX464 = C4*GK464 - GK564*S4;
GX564 = C4*GK564 + GK464*S4;
TKT114 = C4*GX114 - GX124*S4;
TKT214 = C4*GX214 - GX224*S4;
TKT414 = C4*GX414 - GX424*S4;
TKT514 = C4*GX514 - GX524*S4;
TKT614 = C4*GK614 - GK624*S4;
TKT224 = C4*GX224 + GX214*S4;
TKT424 = C4*GX424 + GX414*S4;
TKT524 = C4*GX524 + GX514*S4;
TKT624 = C4*GK624 + GK614*S4;
TKT444 = C4*GX444 - GX454*S4;
TKT544 = C4*GX544 - GX554*S4;
TKT644 = C4*GK644 - GK654*S4;
TKT554 = C4*GX554 + GX544*S4;
TKT654 = C4*GK654 + GK644*S4;
ALJI14 = AP14*C4 - AP24*S4;
ALJI24 = AP14*S4 + AP24*C4;
ALJI44 = AP44*C4 - AP54*S4;
ALJI54 = AP44*S4 + AP54*C4;
MJE113 = M3 + TKT114;
MJE513 = MZ3 + TKT514;
MJE613 = -MY3 + TKT614;
MJE223 = M3 + TKT224;
MJE423 = -MZ3 + TKT424;
MJE623 = MX3 + TKT624;
MJE333 = M3 + M4;
MJE433 = GX434 + MY3;
MJE533 = GX534 - MX3;
MJE443 = TKT444 + XX3;
MJE543 = TKT544 + XY3;
MJE643 = TKT644 + XZ3;
MJE553 = TKT554 + YY3;
MJE653 = TKT654 + YZ3;
MJE663 = GK664 + ZZ3;
VBE13 = -ALJI14 - SW13;
VBE33 = -AP34 - SW33;
VBE43 = -ALJI44 - KW13;
VBE63 = -AP64 - KW33;
GW3 = -FS3*sign(QP3) - FV3*QP3 + GAM3 + VBE33;
JD3 = 1/(IA3 + MJE333);
JU33 = JD3*MJE333;
JU43 = JD3*MJE433;
JU53 = JD3*MJE533;
GK333 = -JU33*MJE333 + MJE333;
GK433 = -JU43*MJE333 + MJE433;
GK533 = -JU53*MJE333 + MJE533;
GK343 = -JU33*MJE433 + MJE433;
GK443 = -JU43*MJE433 + MJE443;
GK543 = -JU53*MJE433 + MJE543;
GK353 = -JU33*MJE533 + MJE533;
GK453 = -JU43*MJE533 + MJE543;
GK553 = -JU53*MJE533 + MJE553;
NG13 = LW13*MJE113;
NG23 = LW13*TKT214;
NG33 = GK333*LW33;
NG43 = GK433*LW33 + LW13*TKT414;
NG53 = GK533*LW33 + LW13*MJE513;
NG63 = LW13*MJE613;
VS33 = GW3*JU33 + NG33;
VS43 = GW3*JU43 + NG43;
VS53 = GW3*JU53 + NG53;
AP13 = NG13 - VBE13;
AP23 = ALJI24 + NG23;
AP33 = -VBE33 + VS33;
AP43 = -VBE43 + VS43;
AP53 = ALJI54 + VS53;
AP63 = NG63 - VBE63;
GX413 = -R3*TKT214 + TKT414;
GX613 = -MJE113*R3 - MJE513;
GX423 = -MJE223*R3 + MJE423;
GX623 = -R3*TKT214 - TKT524;
GX443 = GK443 - MJE423*R3;
GX643 = -GK543 - R3*TKT414;
GX453 = GK453 - R3*TKT524;
GX653 = -GK553 - MJE513*R3;
GX463 = -MJE623*R3 + MJE643;
GX663 = -MJE613*R3 - MJE653;
TKT443 = -GX423*R3 + GX443;
TKT643 = -GX623*R3 + GX643;
TKT663 = -GX613*R3 - GX653;
ALJI43 = -AP23*R3 + AP43;
ALJI63 = -AP13*R3 - AP53;
MJE112 = M2 + MJE113;
MJE512 = MJE613 + MZ2;
MJE612 = GX613 - MY2;
MJE222 = GK333 + M2;
MJE422 = GK433 - MZ2;
MJE622 = -GK533 + MX2;
MJE332 = M2 + MJE223;
MJE432 = -GX423 + MY2;
MJE532 = -MJE623 - MX2;
MJE442 = TKT443 + XX2;
MJE542 = GX463 + XY2;
MJE642 = TKT643 + XZ2;
MJE552 = MJE663 + YY2;
MJE652 = GX663 + YZ2;
MJE662 = TKT663 + ZZ2;
VBE12 = -AP13 - SW12;
VBE22 = -AP33 - SW22;
VBE42 = -ALJI43 - KW12;
VBE52 = -AP63 - KW22;
GW2 = AP23 - FS2*sign(QP2) - FV2*QP2 + GAM2;
JD2 = 1/(IA2 + MJE332);
JU12 = -JD2*TKT214;
JU32 = JD2*MJE332;
JU42 = JD2*MJE432;
JU52 = JD2*MJE532;
JU62 = -GX623*JD2;
GK112 = JU12*TKT214 + MJE112;
GK312 = JU32*TKT214 - TKT214;
GK412 = GX413 + JU42*TKT214;
GK512 = JU52*TKT214 + MJE512;
GK612 = JU62*TKT214 + MJE612;
GK132 = -JU12*MJE332 - TKT214;
GK332 = -JU32*MJE332 + MJE332;
GK432 = -JU42*MJE332 + MJE432;
GK532 = -JU52*MJE332 + MJE532;
GK632 = -GX623 - JU62*MJE332;
GK142 = GX413 - JU12*MJE432;
GK342 = -JU32*MJE432 + MJE432;
GK442 = -JU42*MJE432 + MJE442;
GK542 = -JU52*MJE432 + MJE542;
GK642 = -JU62*MJE432 + MJE642;
GK152 = -JU12*MJE532 + MJE512;
GK352 = -JU32*MJE532 + MJE532;
GK452 = -JU42*MJE532 + MJE542;
GK552 = -JU52*MJE532 + MJE552;
GK652 = -JU62*MJE532 + MJE652;
GK162 = GX623*JU12 + MJE612;
GK362 = GX623*JU32 - GX623;
GK462 = GX623*JU42 + MJE642;
GK562 = GX623*JU52 + MJE652;
GK662 = GX623*JU62 + MJE662;
VS12 = GW2*JU12;
VS32 = GW2*JU32;
VS42 = GW2*JU42;
VS52 = GW2*JU52;
VS62 = GW2*JU62;
AP12 = -VBE12 + VS12;
AP32 = -AP23 + VS32;
AP42 = -VBE42 + VS42;
AP52 = -VBE52 + VS52;
AP62 = ALJI63 + VS62;
GX512 = GK112*R2 + GK512;
GX422 = -MJE222*R2 + MJE422;
GX532 = GK132*R2 + GK532;
GX442 = GK442 - MJE422*R2;
GX542 = GK142*R2 + GK542;
GX552 = GK152*R2 + GK552;
GX462 = GK462 - MJE622*R2;
GX562 = GK162*R2 + GK562;
TKT442 = -GX422*R2 + GX442;
TKT642 = GK642 - MJE622*R2;
TKT552 = GX512*R2 + GX552;
TKT652 = GK612*R2 + GK652;
ALJI42 = AP42 + R2*VBE22;
ALJI52 = AP12*R2 + AP52;
MJE111 = GK112 + M1;
MJE511 = GX512 + MZ1;
MJE611 = GK612 - MY1;
MJE221 = M1 + MJE222;
MJE421 = GX422 - MZ1;
MJE621 = MJE622 + MX1;
MJE331 = GK332 + M1;
MJE431 = GK432 + MY1;
MJE531 = GX532 - MX1;
MJE441 = TKT442 + XX1;
MJE541 = GX542 + XY1;
MJE641 = TKT642 + XZ1;
MJE551 = TKT552 + YY1;
MJE651 = TKT652 + YZ1;
MJE661 = GK662 + ZZ1;
VBE11 = -AP12 - SW11;
VBE21 = -SW21 + VBE22;
VBE41 = -ALJI42 - KW11;
VBE51 = -ALJI52 - KW21;
GW1 = -AP62 - FS1*sign(QP1) - FV1*QP1 + GAM1;
JD1 = 1/(IA1 + MJE661);
JU11 = JD1*MJE611;
JU21 = JD1*MJE621;
JU31 = GK632*JD1;
JU41 = JD1*MJE641;
JU51 = JD1*MJE651;
JU61 = JD1*MJE661;
GK111 = -JU11*MJE611 + MJE111;
GK211 = -JU21*MJE611;
GK311 = GK312 - JU31*MJE611;
GK411 = GK412 - JU41*MJE611;
GK511 = -JU51*MJE611 + MJE511;
GK611 = -JU61*MJE611 + MJE611;
GK121 = -JU11*MJE621;
GK221 = -JU21*MJE621 + MJE221;
GK321 = -JU31*MJE621;
GK421 = -JU41*MJE621 + MJE421;
GK521 = -JU51*MJE621;
GK621 = -JU61*MJE621 + MJE621;
GK131 = GK312 - GK632*JU11;
GK231 = -GK632*JU21;
GK331 = -GK632*JU31 + MJE331;
GK431 = -GK632*JU41 + MJE431;
GK531 = -GK632*JU51 + MJE531;
GK631 = -GK632*JU61 + GK632;
GK141 = GK412 - JU11*MJE641;
GK241 = -JU21*MJE641 + MJE421;
GK341 = -JU31*MJE641 + MJE431;
GK441 = -JU41*MJE641 + MJE441;
GK541 = -JU51*MJE641 + MJE541;
GK641 = -JU61*MJE641 + MJE641;
GK151 = -JU11*MJE651 + MJE511;
GK251 = -JU21*MJE651;
GK351 = -JU31*MJE651 + MJE531;
GK451 = -JU41*MJE651 + MJE541;
GK551 = -JU51*MJE651 + MJE551;
GK651 = -JU61*MJE651 + MJE651;
GK161 = -JU11*MJE661 + MJE611;
GK261 = -JU21*MJE661 + MJE621;
GK361 = GK632 - JU31*MJE661;
GK461 = -JU41*MJE661 + MJE641;
GK561 = -JU51*MJE661 + MJE651;
GK661 = -JU61*MJE661 + MJE661;
VS11 = GW1*JU11;
VS21 = GW1*JU21;
VS31 = GW1*JU31;
VS41 = GW1*JU41;
VS51 = GW1*JU51;
VS61 = GW1*JU61;
AP11 = -VBE11 + VS11;
AP21 = -VBE21 + VS21;
AP31 = AP32 + VS31;
AP41 = -VBE41 + VS41;
AP51 = -VBE51 + VS51;
AP61 = AP62 + VS61;
GX111 = C1*GK111 - GK211*S1;
GX211 = C1*GK211 + GK111*S1;
GX411 = C1*GK411 - GK511*S1;
GX511 = C1*GK511 + GK411*S1;
GX121 = C1*GK121 - GK221*S1;
GX221 = C1*GK221 + GK121*S1;
GX421 = C1*GK421 - GK521*S1;
GX521 = C1*GK521 + GK421*S1;
GX131 = C1*GK131 - GK231*S1;
GX231 = C1*GK231 + GK131*S1;
GX431 = C1*GK431 - GK531*S1;
GX531 = C1*GK531 + GK431*S1;
GX141 = C1*GK141 - GK241*S1;
GX241 = C1*GK241 + GK141*S1;
GX441 = C1*GK441 - GK541*S1;
GX541 = C1*GK541 + GK441*S1;
GX151 = C1*GK151 - GK251*S1;
GX251 = C1*GK251 + GK151*S1;
GX451 = C1*GK451 - GK551*S1;
GX551 = C1*GK551 + GK451*S1;
GX161 = C1*GK161 - GK261*S1;
GX261 = C1*GK261 + GK161*S1;
GX461 = C1*GK461 - GK561*S1;
GX561 = C1*GK561 + GK461*S1;
TKT111 = C1*GX111 - GX121*S1;
TKT211 = C1*GX211 - GX221*S1;
TKT311 = C1*GK311 - GK321*S1;
TKT411 = C1*GX411 - GX421*S1;
TKT511 = C1*GX511 - GX521*S1;
TKT611 = C1*GK611 - GK621*S1;
TKT221 = C1*GX221 + GX211*S1;
TKT321 = C1*GK321 + GK311*S1;
TKT421 = C1*GX421 + GX411*S1;
TKT521 = C1*GX521 + GX511*S1;
TKT621 = C1*GK621 + GK611*S1;
TKT441 = C1*GX441 - GX451*S1;
TKT541 = C1*GX541 - GX551*S1;
TKT641 = C1*GK641 - GK651*S1;
TKT551 = C1*GX551 + GX541*S1;
TKT651 = C1*GK651 + GK641*S1;
ALJI11 = AP11*C1 - AP21*S1;
ALJI21 = AP11*S1 + AP21*C1;
ALJI41 = AP41*C1 - AP51*S1;
ALJI51 = AP41*S1 + AP51*C1;
GU1 = -G3*JU31;
QDP1 = -GU1 + GW1*JD1;
DY11 = -G3*GK312 + MJE611*QDP1;
DY21 = MJE621*QDP1;
DY31 = -G3*MJE331 + GK632*QDP1;
DY41 = -G3*MJE431 + MJE641*QDP1;
DY51 = -G3*MJE531 + MJE651*QDP1;
DY61 = -G3*GK632 + MJE661*QDP1;
E11 = DY11 - VBE11;
E21 = DY21 - VBE21;
E31 = AP32 + DY31;
N11 = DY41 - VBE41;
N21 = DY51 - VBE51;
N31 = AP62 + DY61;
GU2 = -G3*JU32 + JU62*QDP1;
QDP2 = -GU2 + GW2*JD2;
VP32 = -G3 + QDP2;
DY12 = MJE612*QDP1 - TKT214*VP32;
DY22 = MJE622*QDP1;
DY32 = -GX623*QDP1 + MJE332*VP32;
DY42 = MJE432*VP32 + MJE642*QDP1;
DY52 = MJE532*VP32 + MJE652*QDP1;
DY62 = -GX623*VP32 + MJE662*QDP1;
E12 = DY12 - VBE12;
E22 = DY22 - VBE22;
E32 = -AP23 + DY32;
N12 = DY42 - VBE42;
N22 = DY52 - VBE52;
N32 = ALJI63 + DY62;
VR13 = LW13 - QDP1*R3;
GU3 = JU33*LW33 - JU53*QDP1;
QDP3 = -GU3 + GW3*JD3;
ZETA33 = LW33 + QDP3;
DY13 = MJE113*VR13 - MJE513*QDP1 - TKT214*VP32;
DY23 = -MJE223*VP32 - QDP1*TKT524 + TKT214*VR13;
DY33 = MJE333*ZETA33 - MJE533*QDP1;
DY43 = -MJE423*VP32 + MJE433*ZETA33 - MJE543*QDP1 + TKT414*VR13;
DY53 = MJE513*VR13 + MJE533*ZETA33 - MJE553*QDP1 - TKT524*VP32;
DY63 = MJE613*VR13 - MJE623*VP32 - MJE653*QDP1;
E13 = DY13 - VBE13;
E23 = ALJI24 + DY23;
E33 = DY33 - VBE33;
N13 = DY43 - VBE43;
N23 = ALJI54 + DY53;
N33 = DY63 - VBE63;
VR14 = C4*VR13 - S4*VP32;
VR24 = -C4*VP32 - S4*VR13;
VR44 = -QDP1*S4 + WQ14;
VR54 = -C4*QDP1 + WQ24;
GU4 = JU14*VR14 + JU24*VR24 + JU44*VR44 + JU54*VR54;
QDP4 = -GU4 + GW4*JD4;
DY14 = M4*VR14 - MY4*QDP4 + MZ4*VR54;
DY24 = M4*VR24 + MX4*QDP4 - MZ4*VR44;
DY34 = M4*ZETA33 - MX4*VR54 + MY4*VR44;
DY44 = MY4*ZETA33 - MZ4*VR24 + QDP4*XZ4 + VR44*XX4 + VR54*XY4;
DY54 = -MX4*ZETA33 + MZ4*VR14 + QDP4*YZ4 + VR44*XY4 + VR54*YY4;
DY64 = MX4*VR24 - MY4*VR14 + QDP4*ZZ4 + VR44*XZ4 + VR54*YZ4;
E14 = DY14 + SW14;
E24 = DY24 + SW24;
E34 = DY34 + SW34;
N14 = DY44 + KW14;
N24 = DY54 + KW24;
N34 = DY64 + KW34;
%END Symoro%



% Posiciones
sys(1) =  x(5);
sys(2) =  x(6);
sys(3) =  x(7);
sys(4) = x(8);
% velocidades
sys(5) = QDP1;
sys(6) = QDP2;
sys(7) = QDP3;
sys(8) = QDP4;



% end mdlDerivatives
%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u)

  sys(1) = x(1);
  sys(2) = x(2);
  sys(3) = x(3);
  sys(4) = x(4);
  sys(5) = x(5);
  sys(6) = x(6);
  sys(7) = x(7);
  sys(8) = x(8);
    
  
  
% end mdlOutputs