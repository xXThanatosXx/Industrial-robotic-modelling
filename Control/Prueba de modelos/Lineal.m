
% Tiempo de muestreo y duración final de la trayectoria:
Tfinal=3.0;
Tem=0.001;
% Cálculo del número de muestras:
nbech=(Tfinal/Tem)+1;
if ((round(nbech)-nbech) == 0)
instant=[0:Tem:Tfinal]';
else
nbech=nbech+1;
instant=[0:Tem:Tfinal+Tem]';
end
% Definición de la base de tiempo:
instant = instant(1:3000,:);
% Definición de las dos líneas:
t=0;
for h=1:1:1500
t=t+Tem;
x1(h)=t;
y1(h)=t;
end
t=1.5;
for h=1:1:1500
t=t+Tem;
x2(h)=-t + 3.0;
y2(h)= t;
end
xx = [x1 x2];
yy = [y1 y2];
xx = xx';
yy = yy';
%--------------------
cons1= 0.4 + 0.01*xx;
cons2= 0.4 + 0.01*yy;
cons3= 0.4*ones(3000,1);
% save tra cons1 cons2 cons3 instant