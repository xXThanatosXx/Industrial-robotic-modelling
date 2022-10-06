clc
clear 

% Trayectoría (X,Y,Z)
circular;
% lineal;
%------------
% Simulación de MGI y MGD versión 2015
sim('PolarCinematica2015.slx') 
% Simulación de MGI y MGD versión 2022
% sim('PolarCinematica.slx') 
% Gráficas
figure(1)
% grafica la posición en x,y de la trayectoría deseada en colo azul
plot(cons1,cons2,'*','MarkerSize',10,'MarkerEdgeColor','b')
hold on
% grafica la posición en x,y de la trayectoría obtenida en color rojo
plot(ans.X,ans.Y,'-','MarkerSize',10,'MarkerEdgeColor','r')
% axis([0.37 0.425 0.39 .44])
title("Test Módelos Geométricos Directo e Inverso")
xlabel("Eje X")
ylabel("Eje y")
legend("T Buscada","T obtenida")
hold off
grid on

figure(2)
plot(ans.Error)
title("Error cudratico Medio de X,Y,Z")
xlabel("Muestras")
ylabel("Amplitud (m/s)")
grid on



