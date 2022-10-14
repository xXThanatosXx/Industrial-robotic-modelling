clc
clear variables

% Trayectoria (X,Y,Z)
% circular;
Lineal;
% Simulación de MGI y MGD versión 2015
sim('Robot3GDL2015.slx') 
% Simulación de MGI y MGD versión 2022
% sim('Robot3GDL.slx')
figure(1)
% grafica la posición en x,y de la trayectoría deseada en color negro
plot(cons1,cons2,'k+:')
hold on
% grafica la posición en x,y de la trayectoría obtenida en color rojo
plot(ans.X,ans.Y,'r-','LineWidth',1)
axis("equal")
title("Test Módelos Geométricos Directo e Inverso")
xlabel("Eje X")
ylabel("Eje y")
legend("T Buscada","T obtenida")
hold off
grid on
