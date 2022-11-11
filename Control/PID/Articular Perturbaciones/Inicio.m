% Condiciones iniciales
  clear;
  clc;

% Tiempo de muestreo
  Tem = 0.0001 ;

% Articular:
% Posicion inicial de la trayectoria deseada
  QI=[0;0;0;0];
  trayectoria;

% Constantes control dinamico  
% % Constantes control dinamico  coon bloque creado
% [Kp1 Kp2 Kp3 Kp4]
  Kp1 = 200; 
  Kp2 = 200;
  Kp3 = 200;
  Kp4 = 200;
 
% [Kv1 Kv2 Kv3 Kv4]
  Kv1 = 50;
  Kv2 = 50;
  Kv3 = 50;
  Kv4 = 50;
 
%   [Ki1 Ki2 Ki3 Ki4]
  Ki1 = 0 ;
  Ki2 = 0 ;
  Ki3 = 0 ;
  Ki4 = 0 ;
%% Gráficas
figure(1)
plot(instant,qd_1), grid on
title("Posiciones Deseadas")
hold on
plot(instant,qd_2), grid on
plot(instant,qd_3), grid on
plot(instant,qd_4), grid on
axis("equal")
hold off
figure(2)
plot(pos)
title("Posiciones obtenidas")
grid on

  
