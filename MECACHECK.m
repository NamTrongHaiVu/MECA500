function MECACHECK()
robot = UR3;
centerPoint = [0,0,0];
radii = [0.7,0.3,0.3];
[X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );
for i = 1:4
    robot.model.points{i} = [X(:),Y(:),Z(:)];
    warning off
    robot.model.faces{i} = delaunay(robot.model.points{i});    
    warning on;
end
robot.model.plot3d([0,0,0,0,0,0]);
robot.model.teach
% hold on;
% 
% ellipsoidAtOrigin_h = surf(X,Y,Z);
% % Make the ellipsoid translucent (so we can see the inside and outside points)
% alpha(0.1);
% axis equal
% 
% % One side of the cube
% [Y,Z] = meshgrid(-0.75:0.05:0.75,-0.75:0.05:0.75);
% sizeMat = size(Y);
% X = repmat(0.75,sizeMat(1),sizeMat(2));
% oneSideOfCube_h = surf(X,Y,Z);
% 
% 
% cubePoints = [X(:),Y(:),Z(:)];
% 
% % Make a cube by rotating the single side by 0,90,180,270, and around y to make the top and bottom faces
% cubePoints = [ cubePoints ...
%              ; cubePoints * rotz(pi/2)...
%              ; cubePoints * rotz(pi) ...
%              ; cubePoints * rotz(3*pi/2) ...
%              ; cubePoints * roty(pi/2) ...
%              ; cubePoints * roty(-pi/2)];    
% cube_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'b.');
% 
% algebraicDist = GetAlgebraicDist(cubePoints, centerPoint, radii);
% pointsInside = find(algebraicDist < 1);
% display(['2.4: There are ', num2str(size(pointsInside,1)),' points inside']);
% 
% q = [0,0,0,0,0,0];
% tr = robot.model.fkine(q);
% cubePointsAndOnes = [inv(tr) * [cubePoints,ones(size(cubePoints,1),1)]']';
% updatedCubePoints = cubePointsAndOnes(:,1:3);
% algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoint, radii);
% pointsInside = find(algebraicDist < 1);
% display(['There are now ', num2str(size(pointsInside,1)),' points inside']);
% 
% 
% 
% %robot.plot3d([0,0,0,0,0,0]);