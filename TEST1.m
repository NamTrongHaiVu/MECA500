clear all; close all; clc

q = [0,pi/2,0,0,0,0];
MECACHECK(q)
view(3);
camlight;
hold on;
pause(3);
%% Run
centerPoint = [0,0,0];
radii = [3,2,1];
[X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );
view(3);
hold on;

% 2.2
ellipsoidAtOrigin_h = surf(X,Y,Z);
% Make the ellipsoid translucent (so we can see the inside and outside points)
alpha(0.1);

% 2.3
% One side of the cube
[Y,Z] = meshgrid(-0.75:0.05:0.75,-0.75:0.05:0.75);
sizeMat = size(Y);
X = repmat(0.75,sizeMat(1),sizeMat(2));
oneSideOfCube_h = surf(X,Y,Z);

% Combine one surface as a point cloud
cubePoints = [X(:),Y(:),Z(:)];

% Make a cube by rotating the single side by 0,90,180,270, and around y to make the top and bottom faces
cubePoints = [ cubePoints ...
             ; cubePoints * rotz(pi/2)...
             ; cubePoints * rotz(pi) ...
             ; cubePoints * rotz(3*pi/2) ...
             ; cubePoints * roty(pi/2) ...
             ; cubePoints * roty(-pi/2)];         
         
% Plot the cube's point cloud         
cubeAtOigin_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'r.');
cubePoints = cubePoints + repmat([2,0,-0.5],size(cubePoints,1),1);
cube_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'b.');
axis equal

% 2.4
algebraicDist = GetAlgebraicDist(cubePoints, centerPoint, radii);
pointsInside = find(algebraicDist < 1);
display(['2.4: There are ', num2str(size(pointsInside,1)),' points inside']);

% 2.5
centerPoint = [1,1,1];
algebraicDist = GetAlgebraicDist(cubePoints, centerPoint, radii);
pointsInside = find(algebraicDist < 1);
display(['2.5: There are now ', num2str(size(pointsInside,1)),' points inside']);

% 2.6
centerPoint = [0,0,0];
cubePointsAndOnes = [inv(transl(1,1,1)) * [cubePoints,ones(size(cubePoints,1),1)]']';
updatedCubePoints = cubePointsAndOnes(:,1:3);
algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoint, radii);
algebraicDist = GetAlgebraicDist(cubePoints, centerPoint, radii);          
pointsInside = find(algebraicDist < 1);
display(['2.6: There are now ', num2str(size(pointsInside,1)),' points inside']);

% 2.7
centerPoint = [0,0,0];
cubePointsAndOnes = [inv(transl(1,1,1)*trotx(pi/4)) * [cubePoints,ones(size(cubePoints,1),1)]']';
updatedCubePoints = cubePointsAndOnes(:,1:3);
algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoint, radii);
pointsInside = find(algebraicDist < 1);
display(['2.7: There are now ', num2str(size(pointsInside,1)),' points inside']);
pause(1);
keyboard
try delete(cubeAtOigin_h); end;
try delete(ellipsoidAtOrigin_h); end;
try delete(oneSideOfCube_h); end;

centerPoint = [0,0,0];
radii = [1,0.5,0.5];
[X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );
for i = 1:4
    robot.my3.points{i} = [X(:),Y(:),Z(:)];
    warning off
    robot.my3.faces{i} = delaunay(robot.my3.points{i});    
    warning on;
end

q = [0,0,0,0,0,0]
tr = robot.my3.fkine(q);
cubePointsAndOnes = [inv(tr) * [cubePoints,ones(size(cubePoints,1),1)]']';
updatedCubePoints = cubePointsAndOnes(:,1:3);
algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoint, radii);
pointsInside = find(algebraicDist < 1);
display(['2.9: There are now ', num2str(size(pointsInside,1)),' points inside']);


%% Save
% %% 1.2
% % Plane params (to work out distances for questions)
% createQuestion = true; % For creating the lab exercise question
% if createQuestion
%     pNormal = [-0.3090, 0.9511, 0];            % Create questions
%     pPoint = [0,3,0];                          % Create questions
% end
% tr = robot.my3.fkine(q)
% startP = tr(1:3,4)';
% if createQuestion
%     endP = tr(1:3,4)' + 10 * tr(1:3,3)';                             % Create questions
%     intersectP = LinePlaneIntersection(pNormal,pPoint,startP,endP);  % Create questions
%     dist = dist2pts(startP,intersectP)                               % Create questions
% else    
%     dist = 1.9594;
% end
% endP = tr(1:3,4)' + dist * tr(1:3,3)';
% line1_h = plot3([startP(1),endP(1)],[startP(2),endP(2)],[startP(3),endP(3)],'r');
% plot3(endP(1),endP(2),endP(3),'r*');
% axis equal;
% verts = endP;
% pause(1);
% % try delete(line1_h); end
% 
% %% 1.3
% % New pose 1
% q = [pi/10,pi/2,0,0,0,0];
% tr = robot.my3.fkine(q);
% robot.my3.animate(q);
% startP = tr(1:3,4)';
% if createQuestion
%     endP = tr(1:3,4)' + 10 * tr(1:3,3)';                             % Create questions
%     intersectP = LinePlaneIntersection(pNormal,pPoint,startP,endP);  % Create questions
%     dist = dist2pts(startP,intersectP)                               % Create questions
% else    
%     dist = 2.4861;
% end
% endP = tr(1:3,4)' + dist * tr(1:3,3)';
% line_h(2) = plot3([startP(1),endP(1)],[startP(2),endP(2)],[startP(3),endP(3)],'r');
% plot3(endP(1),endP(2),endP(3),'r*');
% verts = [verts; endP];
% pause(1);
% 
% % New pose 2
% q = [-pi/10,5*pi/12,0,0,0,0];
% tr = robot.my3.fkine(q);
% robot.my3.animate(q);
% startP = tr(1:3,4)';
% if createQuestion
%     endP = tr(1:3,4)' + 10 * tr(1:3,3)';                            % Create questions
%     intersectP = LinePlaneIntersection(pNormal,pPoint,startP,endP); % Create questions
%     dist = dist2pts(startP,intersectP)                              % Create questions
% else
%     dist = 1.9132;
% end
% endP = tr(1:3,4)' + dist * tr(1:3,3)';
% line_h(3) = plot3([startP(1),endP(1)],[startP(2),endP(2)],[startP(3),endP(3)],'r'); %#ok<NASGU>
% plot3(endP(1),endP(2),endP(3),'r*');
% verts = [verts; endP];
% axis equal
% pause(1);