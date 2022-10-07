 clear all; close all; clc            
%  GT1 = Gripper; 
%  GT2 = Gripper2;
%  r3 = LinearMYUR3
%     hold on
%     PlaceObject('smalltube.ply',[0.2,0,0.35]);
%    % PlaceObject('minitube.ply');
%     %PlaceObject('grippermeca500test.ply',[-0.16,-0.2,0.35]);
%     axis equal
%%

centerPoint = [1,2,-3];
radii = [1,2,3];
[X,Y,Z] = ellipsoid( -5:1:5,-5:1:5 );
view(3);
hold on;







%%
% q1 = [0 -90 70 0 0 0 ]*pi/180;
% q2 = [-115 90 -15 -60 70 0 ]*pi/180;
% 
% steps = 1000;
% hold on
% for i = 1:steps
%     qMatrix = jtraj(q1,q2,steps);
%     robot.my3.animate(qMatrix(i,:));  
%     drawnow()
% end