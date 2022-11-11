clear; 
clc;
% %Definir tiempo de muestreo
Tem = 0.001;
circular;
% Lineal;
% %Definir posición inicial para las articulaciones del robot
% %Para el Control Cartesiano:
QI =mgi(cons1(1),cons2(1),cons3(1));


% [Kp1 Kp2 Kp3 Kp4]
  Kp1 = 100; 
  Kp2 = 100;
  Kp3 = 100;
  Kp4 = 100;
 
% [Kv1 Kv2 Kv3 Kv4]
  Kv1 = 10;
  Kv2 = 10;
  Kv3 = 10;
  Kv4 = 10;
 
%   [Ki1 Ki2 Ki3 Ki4]
  Ki1 = 0 ;
  Ki2 = 0 ;
  Ki3 = 0 ;
  Ki4 = 0 ;


%% Graficar Resultados 

figure
plot(cons1,cons2,'k+:')
hold on
plot(x,y,'r-','LineWidth',1)
axis equal
title("Respuesta a trayectoría Lineal")
xlabel("Eje X")
ylabel("Eje y")
legend("T-L Referencia","T-L Obtenida")
hold off
grid on




