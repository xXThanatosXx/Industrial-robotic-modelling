clear; 
clc;
% %Definir tiempo de muestreo
Ts = 0.001;
% Trayectoría Lineal y Circular 
% Lineal;
circular;

% %Definir posición inicial para las articulaciones del robot
% %Para el Control Cartesiano:
QI =mgi(cons1(1),cons2(1),cons3(1));


% Constantes control dinamico  
% GANANCIA PROPORCIONAL
KP1 =100000;
KP2 =85000;
KP3 =70000;
KP4 =70000;


% GANANCIA DERIVATIVA
KV1=  360;
KV2=  280; 
KV3=  120; 
KV4=  120;
