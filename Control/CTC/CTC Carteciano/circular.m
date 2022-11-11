%----Movimiento circular-----------
Tfinal=3.0;
Ts=0.001;

% Calculo del numero de muestras %

nbech=(Tfinal/Ts)+1;
if ((round(nbech)-nbech) == 0)
	instant=[0:Ts:Tfinal]'; 
else
	nbech=nbech+1;
	instant=[0:Ts:Tfinal+Ts]';
end
t=0;

for h=1:1:nbech
   t=t+Ts;
   x1(h)=0.02*sin(2*t*pi/Tfinal);% - 1.266);%2*pi/5 = 1.2566
   y1(h)=0.02*cos(2*t*pi/Tfinal);% - 1.266);
end
x1=x1';
y1=y1';
%--------------------
cons1= 0.4 + x1;
cons2= 0.4 + y1;
[f,c]=size(cons1);
cons3=0.5*ones(f,1);