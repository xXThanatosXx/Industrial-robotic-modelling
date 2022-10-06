clc
clear variables
close all

% My robot
% Longitudes iniciales
Q1 = 0; %rad
R2=0.363; %cm
R3=0.363; %cm
alpha= -pi/2; %rad  

% Definicion del robot usando DH (th,d,a,alfa,P/R)
L(1) = Link([Q1,0,0,0,0],'modified'); 
L(2) = Link([0,0,0,0,1],'modified');
L(2).qlim = [0,R2];%valor de desplazamiento de articulación prismpatica
L(3) = Link([0,0,0,alpha,1],'modified');
L(3).qlim = [0,R3];

RPP = SerialLink(L,'name','RPP') %xmin xmax, ymin ymax, zmin zmax 
RPP.teach([0.3 0.22 0.23],'workspace',[-0.7 1 -1 1 -0.3 0.7])
 
% % Test MGD
P= zeros(3,1);

% while(true)
%     disp('Calculo de MGD')
%     prompt = 'Desea continuar (s) ?: ';
%     r=input(prompt,'s');
%     if r== 's'
%         q = RPP.getpos();
%         P =Cilindric_mgd(q(1),q(2),q(3));
%         disp('Px Py Pz')
%         disp(P)
%     else 
%         close all
%         break 
%     end
% 
% end

% Puntos
% [0.3,0.3,0.12]
Q= zeros(3,1);
% Test MGi
while(true)
    disp('Calculo de MGI')
    prompt = 'Desea calcular Q1, R2,R3 (s) ?: '
    r=input(prompt,'s');
    if r== 's'
        P=input('Ingrese [x,y,z] deseada');
        plot_sphere(P,0.02,'y');
        input('Presiona una tecla para mover el robot')
        Q = MGI(P(1), P(2),P(3));
        RPP.plot(Q,'workspace',[-0.7 1 -1 1 -0.3 0.7])
        disp('Q1 R2 R3')
        disp([rad2deg(Q(1)) Q(2) Q(3)])
    else
        close all
        break
    end
end



