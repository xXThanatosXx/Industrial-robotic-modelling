clc
clear 

% Trayectoria (X,Y,Z)
circular;
% lineal;
% Trayectori;
sim('PolarCinematica.slx') 
% 
figure(1)
plot(cons1,cons2,'*','MarkerSize',10,'MarkerEdgeColor','b')
hold on
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



