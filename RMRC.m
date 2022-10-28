clear all
close all
clc

camlight;
axis([-2,2,-2,2,-1,2])
view(3);

robot = Meca500;  

% q0 = robot.my3.getpos();
% % Initial position of 3 tubes
I1 = transl([-0.5 -0.2 0.1]) * troty(-pi/2) ; 
% % I2 = transl([0.6 0.2 0]) * trotx(pi);
% % I3 = transl([0.9 0.2 0]) * trotx(pi);
% % Ini_pos = [I1 ; I2 ; I3];
% 
% % End position of 3 tubes
% E1 = transl([-0.2 0.8 0.7]) * troty(-pi/2);
% % q = zeros(1,6);
% q1 = robot.my3.ikcon(I1);
% % q1 = robot.my3.ikine(I1, q, [1 1 1 0 0 0]);
% q1_move = jtraj(q0,q1,50);
% for i = 1:50
%     robot.my3.animate(q1_move(i,:));
%     drawnow();
% end

% 3.5: Resolved Motion Rate Control
steps = 50;

% 3.6
%x1 = [-0.5 -0.2 0.1 -1 0 0]';
x1 = [0 0 0 0 0 0]';
%x2 = [0 0.4 0.3 -1 0 0]';
x2 = [-0.5 0.2 0.1 -1 0 0]';

deltaT = 0.05;                                 % Discrete time step

% 3.7
x = zeros(6,steps);
s = lspb(0,1,steps);                           % Create interpolation scalar
for i = 1:steps
    x(:,i) = x1*(1-s(i)) + s(i)*x2;            % Create trajectory in x-y plane
end
keyboard
% 3.8
qMatrix = zeros(steps,robot.my3.n);
% M = [1 1 1 0 0 0];

% 3.9
% T1 = transl(-0.5,-0.2,0.1);
% T1 = robot.my3.fkine(robot.my3.getpos);
% qMatrix(1,:) = robot.my3.ikine(I1,[0 0 0 0 0 0],M);   % Solve for joint angles
qMatrix(1,:) = robot.my3.ikcon(I1);
hold on
% 3.10
for i = 1:steps-1
    xdot = (x(:,i+1) - x(:,i))/deltaT;                  % Calculate velocity at discrete time step
    J = robot.my3.jacob0(qMatrix(i,:));               % Get the Jacobian at the current state
%     J = J(1:2,:);                                     % Take only first 2 rows
    qdot = inv(J)*xdot;                                 % Solve velocitities via RMRC
    qMatrix(i+1,:) =  qMatrix(i,:) + deltaT*qdot';      % Update next joint state
%     Temp = robot.my3.fkine(qMatrix(i+1,:));
%     Temp = Temp(1:3,4)';
%     plot3(Temp(1),Temp(2),Temp(3),'o');
    
    robot.my3.animate(qMatrix(i,:));
    drawnow();
end
% robot.my3.plot(qMatrix);