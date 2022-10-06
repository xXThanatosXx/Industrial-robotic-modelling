function x = mgd_Polar(q1,q2,d3)

% Modelo geométrico directo del SCARA de cuatro ejes:
 
% Valores constantes:
d1 = 1;
 
% Matriz de transformación 0T3 (modelo geométrico directo):
 

TTT=  [cos(q1)*cos(q2), -sin(q1), -cos(q1)*sin(q2), -d3*cos(q1)*sin(q2); 
        cos(q2)*sin(q1), cos(q1), -sin(q1)*sin(q2), -d3*sin(q1)*sin(q2); 
        sin(q2), 0, cos(q2), d1 + d3*cos(q2); 
        0, 0, 0, 1];

 
% Valores de la cuarta columna:
xa = TTT(1,4);
ya = TTT(2,4);
za = TTT(3,4);
 
x = [xa ya za];
