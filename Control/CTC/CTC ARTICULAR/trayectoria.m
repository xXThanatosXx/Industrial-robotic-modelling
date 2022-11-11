%--------------------------------------------
% Génération d'une consigne de type Bang_Bang  
%--------------------------------------------

format long e
global instant consigne Tech Tfin cons_vit

%--------------
Tech=0.001;
Tech_com=Tech;

% Eje 1:

Qinicial = 0.0;
Qfinal = 1.0;

%Aceleracion deseada
Kai=10;
%Desaceleracion deseada
Kdi=10;
% Velocidad maxima deseada
Kvi=5;
% Duracion
Tfinal=1.0;
	
qpprec=0;
	
%%%%%%%%%%%%%%%%%%%%%%% ZONA DE INICIALIZACION %%%%%%%%%%%%%%%%%%%%
	
% Calculo de la distancia a recorrer
	
delta_pos=Qfinal-Qinicial;
	
Vlim=sqrt(Kai*delta_pos);
if ( Kvi >= Vlim )
	Kvi=Vlim;
end 
	
% Puntos de quiebre:
	
t1=0;
t3=(round(delta_pos/Kvi/Tech))*Tech;
Kvi=delta_pos/t3;
t2=(round(Kvi/Kai/Tech))*Tech;
Kai=Kvi/t2;
t4=t2+t3;

% Calculo del numero de muestras:
	
nbech=(Tfinal/Tech)+1;
if ((round(nbech)-nbech) == 0)
	instant=[0:Tech:Tfinal]'; 
else
	nbech=nbech+1;
	instant=[0:Tech:Tfinal+Tech]';
end
	 
% Inicializacion de vectores:
	
xt=0;
vt=0;
at=0;
	
temps=0;
v=[]';
p=[]';
a=[]';
	
% Construccion de los vectores para la simulacion:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	
for g=1:1:nbech
	
	v(g)=vt;
	p(g)=xt;
	a(g)=at;
	  
	if (temps<=t2)
    		ti=t1;
		a0=0;
		a1=0;
		a2=Kai/2;
	elseif (temps<=t3)
		ti=t2;
		a0=0.5*Kvi^2/Kai;
		a1=Kvi;
		a2=0;
	elseif (temps<=t4)
		ti=t3;
		a0=delta_pos-(0.5*Kvi^2/Kai);
		a1=Kvi;
		a2=-0.5*Kai;
	else
		ti=t4;
		a0=delta_pos;
		a1=0;
		a2=0;
	end
	
	delta_t=temps-ti;
	xt=a0+a1*delta_t+a2*delta_t*delta_t;
	vt=a1+2*a2*delta_t;
	at=2*a2;
	temps=temps+Tech;
end
	
qd_1 = p';
qpd_1 = v';
qppd_1 = a';

% Eje 2:

Qinicial = 0.0;
Qfinal=1.5;

%Aceleracion deseada
Kai=20;
%Desaceleracion deseada
Kdi=20;
% Velocidad maxima deseada
Kvi=25;
% Duracion
Tfinal=1.0;
	
qpprec=0;
	
%%%%%%%%%%%%%%%%%%%%%%% ZONA DE INICIALIZACION %%%%%%%%%%%%%%%%%%%%
	
% Calculo de la distancia a recorrer
	
delta_pos=Qfinal-Qinicial;
	
Vlim=sqrt(Kai*delta_pos);
if ( Kvi >= Vlim )
	Kvi=Vlim;
end 
	
% Puntos de quiebre:
	
t1=0;
t3=(round(delta_pos/Kvi/Tech))*Tech;
Kvi=delta_pos/t3;
t2=(round(Kvi/Kai/Tech))*Tech;
Kai=Kvi/t2;
t4=t2+t3;

% Calculo del numero de muestras:
	
nbech=(Tfinal/Tech)+1;
if ((round(nbech)-nbech) == 0)
	instant=[0:Tech:Tfinal]'; 
else
	nbech=nbech+1;
	instant=[0:Tech:Tfinal+Tech]';
end
	 
% Inicializacion de vectores:
	
xt=0;
vt=0;
at=0;
	
temps=0;
v=[]';
p=[]';
a=[]';
	
% Construccion de los vectores para la simulacion:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	
for g=1:1:nbech
	
	v(g)=vt;
	p(g)=xt;
	a(g)=at;
	  
	if (temps<=t2)
    		ti=t1;
		a0=0;
		a1=0;
		a2=Kai/2;
	elseif (temps<=t3)
		ti=t2;
		a0=0.5*Kvi^2/Kai;
		a1=Kvi;
		a2=0;
	elseif (temps<=t4)
		ti=t3;
		a0=delta_pos-(0.5*Kvi^2/Kai);
		a1=Kvi;
		a2=-0.5*Kai;
	else
		ti=t4;
		a0=delta_pos;
		a1=0;
		a2=0;
	end
	
	delta_t=temps-ti;
	xt=a0+a1*delta_t+a2*delta_t*delta_t;
	vt=a1+2*a2*delta_t;
	at=2*a2;
	temps=temps+Tech;
end
	
qd_2 = p';
qpd_2 = v';
qppd_2 = a';

% Eje 3:

Qinicial = 0.0;
Qfinal=1.3;

%Aceleracion deseada
Kai=15;
%Desaceleracion deseada
Kdi=15;
% Velocidad maxima deseada
Kvi=10;
% Duracion
Tfinal=1.0;
	
qpprec=0;
	
%%%%%%%%%%%%%%%%%%%%%%% ZONA DE INICIALIZACION %%%%%%%%%%%%%%%%%%%%
	
% Calculo de la distancia a recorrer
	
delta_pos=Qfinal-Qinicial;
	
Vlim=sqrt(Kai*delta_pos);
if ( Kvi >= Vlim )
	Kvi=Vlim;
end 
	
% Puntos de quiebre:
	
t1=0;
t3=(round(delta_pos/Kvi/Tech))*Tech;
Kvi=delta_pos/t3;
t2=(round(Kvi/Kai/Tech))*Tech;
Kai=Kvi/t2;
t4=t2+t3;

% Calculo del numero de muestras:
	
nbech=(Tfinal/Tech)+1;
if ((round(nbech)-nbech) == 0)
	instant=[0:Tech:Tfinal]'; 
else
	nbech=nbech+1;
	instant=[0:Tech:Tfinal+Tech]';
end
	 
% Inicializacion de vectores:
	
xt=0;
vt=0;
at=0;
	
temps=0;
v=[]';
p=[]';
a=[]';
	
% Construccion de los vectores para la simulacion:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	
for g=1:1:nbech
	
	v(g)=vt;
	p(g)=xt;
	a(g)=at;
	  
	if (temps<=t2)
    		ti=t1;
		a0=0;
		a1=0;
		a2=Kai/2;
	elseif (temps<=t3)
		ti=t2;
		a0=0.5*Kvi^2/Kai;
		a1=Kvi;
		a2=0;
	elseif (temps<=t4)
		ti=t3;
		a0=delta_pos-(0.5*Kvi^2/Kai);
		a1=Kvi;
		a2=-0.5*Kai;
	else
		ti=t4;
		a0=delta_pos;
		a1=0;
		a2=0;
	end
	
	delta_t=temps-ti;
	xt=a0+a1*delta_t+a2*delta_t*delta_t;
	vt=a1+2*a2*delta_t;
	at=2*a2;
	temps=temps+Tech;
end
	
qd_3 = p';
qpd_3 = v';
qppd_3 = a';

% Eje 4:

Qinicial = 0.0;
Qfinal=0.80;

%Aceleracion deseada
Kai=10;
%Desaceleracion deseada
Kdi=10;
% Velocidad maxima deseada
Kvi=5;
% Duracion
Tfinal=1.0;
	
qpprec=0;
	
%%%%%%%%%%%%%%%%%%%%%%% ZONA DE INICIALIZACION %%%%%%%%%%%%%%%%%%%%
	
% Calculo de la distancia a recorrer
	
delta_pos=Qfinal-Qinicial;
	
Vlim=sqrt(Kai*delta_pos);
if ( Kvi >= Vlim )
	Kvi=Vlim;
end 
	
% Puntos de quiebre:
	
t1=0;
t3=(round(delta_pos/Kvi/Tech))*Tech;
Kvi=delta_pos/t3;
t2=(round(Kvi/Kai/Tech))*Tech;
Kai=Kvi/t2;
t4=t2+t3;

% Calculo del numero de muestras:
	
nbech=(Tfinal/Tech)+1;
if ((round(nbech)-nbech) == 0)
	instant=[0:Tech:Tfinal]'; 
else
	nbech=nbech+1;
	instant=[0:Tech:Tfinal+Tech]';
end
	 
% Inicializacion de vectores:
	
xt=0;
vt=0;
at=0;
	
temps=0;
v=[]';
p=[]';
a=[]';
	
% Construccion de los vectores para la simulacion:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	
for g=1:1:nbech
	
	v(g)=vt;
	p(g)=xt;
	a(g)=at;
	  
	if (temps<=t2)
    		ti=t1;
		a0=0;
		a1=0;
		a2=Kai/2;
	elseif (temps<=t3)
		ti=t2;
		a0=0.5*Kvi^2/Kai;
		a1=Kvi;
		a2=0;
	elseif (temps<=t4)
		ti=t3;
		a0=delta_pos-(0.5*Kvi^2/Kai);
		a1=Kvi;
		a2=-0.5*Kai;
	else
		ti=t4;
		a0=delta_pos;
		a1=0;
		a2=0;
	end
	
	delta_t=temps-ti;
	xt=a0+a1*delta_t+a2*delta_t*delta_t;
	vt=a1+2*a2*delta_t;
	at=2*a2;
	temps=temps+Tech;
end
	
qd_4 = p';
qpd_4 = v';
qppd_4 = a';

