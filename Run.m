
clear all; close all; clc
robot = UR3;
q = [0 0 0 0 0 0];

%% Create object
centerpnt = [-0.4,0,0.2];
side = 0.3;
plotOptions.plotFaces = true;
[vertex,faces,faceNormals] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);
axis equal

%% Get the transform of every joint
% tr = zeros(4,4,robot.model.n+1);
% tr(:,:,1) = robot.model.base;
% L = robot.model.links;
% for i = 1 : robot.model.n
%     tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
% end

%% Go through each link and also each triangle face
% q1 = [-pi/2,0,0,pi,-pi/2,0];
% qmid = [0 -90 150 120 -900]*pi/180;
% q2 = [40 0 0 180 -90 0]*pi/180;

% while ~isempty(find(1 < abs(diff(rad2deg(jtraj(q1,q2,steps)))),1))
%     steps = steps + 1;
% end
%qMatrix = jtraj(q1,q2,steps);
% 
% result = true(steps,1);
% for i = 1: steps
%     result(i) = IsCollision(robot.model,qMatrix(i,:),faces,vertex,faceNormals,false);
%     robot.model.animate(qMatrix(i,:));
%     drawnow
% end

%% Random
q1 = [-pi/2,0,0,pi,-pi/2,0];
qmid = [0 -90 150 120 -900]*pi/180;
q2 = [40 0 0 180 -90 0]*pi/180;
steps = 300;

robot.model.animate(q1);
qWaypoints = [q1;q2];
isCollision = true;
checkedTillWaypoint = 1;
qMatrix(:,:,10) = [];
while (isCollision)
    startWaypoint = checkedTillWaypoint;
    for i = startWaypoint:size(qWaypoints,1)-1
        qMatrixJoin = InterpolateWaypointRadians(qWaypoints(i:i+1,:),deg2rad(10));
        if ~IsCollision(robot.model,qMatrixJoin,faces,vertex,faceNormals)
            qMatrix = [qMatrix; qMatrixJoin]; %#ok<AGROW>
            robot.model.animate(qMatrixJoin);
            drawnow
            pause(0.01)
            size(qMatrix);
            isCollision = false;
            checkedTillWaypoint = i+1;
            % Now try and join to the final goal (q2)
            qMatrixJoin = InterpolateWaypointRadians([qMatrix(end,:); q2],deg2rad(10));
            if ~IsCollision(robot.model,qMatrixJoin,faces,vertex,faceNormals)
                qMatrix = [qMatrix;qMatrixJoin];
                % Reached goal without collision, so break out
                disp('break here')
                break;
            end
        else
            % Randomly pick a pose that is not in collision
            qRand = (2 * rand(1,6) - 1) * pi;
            while IsCollision(robot.model,qRand,faces,vertex,faceNormals)
                qRand = (2 * rand(1,6) - 1) * pi;
            end
            qWaypoints =[ qWaypoints(1:i,:); qRand; qWaypoints(i+1:end,:)];
            isCollision = true;
            break;
        end
    end
end

% robot.model.animate(qMatrix)
% drawnow

