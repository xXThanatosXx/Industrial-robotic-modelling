clc
clear variables

% Trayctoria (X,Y,Z)
circular;
% Lineal;
% % % 
sim('Robot3GDL.slx') 
figure(1)
plot(cons1,cons2,'k+:')
hold on
plot(ans.X,ans.Y,'r-','LineWidth',1)
axis("equal")
title("Test Módelos Geométricos Directo e Inverso")
xlabel("Eje X")
ylabel("Eje y")
legend("T Buscada","T obtenida")
hold off
grid on


