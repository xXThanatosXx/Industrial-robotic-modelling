% Condiciones iniciales
  clear clc ;

% Tiempo de muestreo
  Tem = 0.001 ;
  Tfinal=1.0;
% Articular:
% Posicion inicial de la trayectoria deseada
  QI=[0;0;0;0];
  trayectoria;

% Constantes control dinamico  
% GANANCIA PROPORCIOANL
KP1 =100000;
KP2 =85000;
KP3 =70000;
KP4 =70000;


% GANANCIA DERIVATIVA
KV1=  360;
KV2=  280; 
KV3=  120; 
KV4=  120; 

 
  

