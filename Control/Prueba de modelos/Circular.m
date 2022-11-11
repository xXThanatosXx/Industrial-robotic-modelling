%----Movimiento circular-----------
Tfinal=20;
Tem=0.001;

% Calculo del numero de muestras %
nbech=(Tfinal/Tem)+1;
if ((round(nbech)-nbech) == 0)
	instant=[0:Tem:Tfinal]'; 
else
	nbech=nbech+1;
	instant=[0:Tem:Tfinal+Tem]';
end
t=0;

for h=1:1:nbech
   t=t+Tem;
   x1(h)=0.1*sin(2*t*pi/Tfinal); %radio de 2 cm
   y1(h)=0.1*cos(2*t*pi/Tfinal);
end
x1=x1';
y1=y1';
%--------------------
cons1= 0.4 + x1; %posición del circulo en el eje x
cons2= 0.4 + y1;%posición del circulo en el eje y
[f,c]=size(cons1);
cons3=0.5*ones(f,1); %posición del circulo en el eje Z a 50 Cm de la base del robot 
% 