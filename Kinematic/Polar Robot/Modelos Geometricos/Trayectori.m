clc
clear
close all
% valores x,y,z 
% initial_points = [0 0 0.4515]
% wayPoints=[initial_points;...
%     (rand*0.15)+0.05 ((rand*4)-2)/10 (rand*0.35)+0.05;...
%     (rand*0.15)+0.05 ((rand*4)-2)/10 (rand*0.35)+0.05;...
%     (rand*0.15)+0.05 ((rand*4)-2)/10 (rand*0.35)+0.05;...
%     (rand*0.15)+0.05 ((rand*4)-2)/10 (rand*0.35)+0.05;...
%     initial_points];

adStartPoint    = [0 0 0.4515];
adBallPoint     = [0.1 0.2 0.05];
adPoint         = [0.15 0 0.25];
adGoalPoint     = [0.1 -0.2 0.05];
adEndPoint      = [0.2452 0 0.2562];

wayPoints     = [adStartPoint; adBallPoint; adPoint; adGoalPoint; adEndPoint];

% 
% figure(1)
% plot3(wayPoints(:,1),wayPoints(:,2),wayPoints(:,3),'.','MarkerSize',20,  'MarkerEdgeColor','k'); %punti di passaggio dell'EE
% hold on
% plot(wayPoints)
traj = cscvn(wayPoints');
% fnplt(traj,'r',2)
% grid on
% hold off


[n,~]= size(wayPoints);
totalPoints = n*30;
x = linspace(0,traj.breaks(end),totalPoints);
eePost = ppval(traj,x);
% 
% plot3(eePost(1,:),eePost(2,:),eePost(3,:),'.','MarkerSize',20,  'MarkerEdgeColor','b'); %punti di passaggio dell'EE
% title('Trayectoria Uno')
% xlabel('x')
% ylabel('y')
% grid on
% hold off 

instant = x';
Tfinal =  x(end);
Tem=Tfinal/length(x);
cons1 = eePost(1,:)';
cons2 = eePost(2,:)';
cons3 = eePost(3,:)';