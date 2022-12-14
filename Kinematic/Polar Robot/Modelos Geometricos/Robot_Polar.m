clc
clear 
close all

Q1= 0; Q2=0; %rad
d1 = 1; d3 =1; %cm
%         (theta  d    a   alpha)
L(1) = Link([Q1, d1,   0,   pi/2,0],'standard');
L(2) = Link([Q2,  0,   0,  -pi/2,0],'standard');
L(3) = Link([ 0,  d3,   0,   0,1],'standard');
L(3).qlim =[0,1];

RPolar = SerialLink(L,'name','Polar') 
% % RPolar.plot([0 0 1],'workspace',[-1 2 -2 2 -2 2])
q0=[0.5 0.5 0.5];
RPolar.teach(q0,'workspace',[-2 2 -2 2 -0.5 2])
% 
% 
% % % Test MGD
P= zeros(0,1);

% while(true)
%     disp('Calculo de MGD')
%     prompt = 'Desea continuar (s) ?: ';
%     r=input(prompt,'s');
%     if r== 's'
%         q = RPolar.getpos();
%         P= mgd_Polar(q(1),q(2),q(3));
%         disp('Px Py Pz')
%         disp(P)
%     else 
%         close all
%         break 
%     end
% end

% Punto a evaluar
% [0.5,0.5,0.3]
while(true)
    disp('Calculo de MGI')
    prompt = 'Desea calcular Q1, Q2,D3 ?. Presione (s) para continuar: '
    r=input(prompt,'s');
    if r== 's'
        P=input('Ingrese la posición [x,y,z] deseada: ');
        plot_sphere([P(1), P(2), P(3)],0.08,'y');
        input('Presiona una tecla para mover el robot')
        RPolar.plot(mgi_Polar(P(1), P(2), P(3)),'workspace',[-2 2 -2 2 -0.5 2]);
        l= RPolar.getpos();
%         disp('Posicion en grados')
%         rad2deg(l);
        disp('Posicion Obtenida x,y,z')
        T= RPolar.fkine(l);
        disp([T.t(1),T.t(2),T.t(3)]);
        disp('error cartesiano');
        error = [P(1)-T.t(1),P(2)-T.t(2),P(3)-T.t(3)];
        disp(error);
        
    else
        close all
        break
    end

end




