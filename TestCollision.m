clear all; close all; clc

robot = UR3;
q = zeros(1,6);                                                     % Create a vector of initial joint angles        

centerpnt = [-0.5,0.25,0.2];
side = 0.5;
plotOptions.plotFaces = true;
[vertex,faces,faceNormals] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);
axis equal

q1 = [15*pi/180, 0, 0, 0, 0, 0];
q2 = [-150*pi/180, 0, 0, 0, 0, 0];

qWaypoints = [q1 ...
     ; deg2rad([0,-79,0,0,0,0]) ...
     ; deg2rad([-130,-79,0,0,0,0]) ... 
    ; q2];
qMatrix = InterpolateWaypointRadians(qWaypoints,deg2rad(5));

if IsCollision(robot.model,qMatrix,faces,vertex,faceNormals)
    error('Collision detected!!');
end
for i = 1:110
    robot.model.animate(qMatrix(i,:));
    drawnow();
end