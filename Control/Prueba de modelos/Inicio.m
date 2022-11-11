clc
clear all



% % Trayectorias
% Lineal
Circular;
sim('ROBOT.slx') 
figure(2)
plot(cons1,cons2,'k+:')
hold on
plot(ans.X,ans.Y,'r-','LineWidth',1)
title("Test Módelos Geométricos Directo e Inverso")
xlabel("Eje X")
ylabel("Eje y")
legend("T Buscada","T obtenida")
hold off
grid on






