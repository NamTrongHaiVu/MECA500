%% Robotics
% Lab 7 
function Lab7Question2Skeleton()

close all;
clear all;
clc;

%% Question 2: Dealing with Singularities

%% 2.1 Load a 2-Link planar robot, and assign parameters for the simulation
mdl_planar2;                                                               % Load 2-Link Planar Robot
M = [1 1 zeros(1,4)];                                                      % Masking Matrix
t = 5 ;                                                                  % Total time in seconds (try 5 sec)
steps = 100 ;                                                              % No. of steps (try 100)
deltaT = t/steps;                                                          % Discrete time step
deltaTheta = 4*pi/steps;                                                   % Small angle change
qMatrix = zeros(steps,2);                                                  % Assign memory for joint angles
x = zeros(2,steps);                                                        % Assign memory for trajectory
m = zeros(1,steps);                                                        % For recording measure of manipulability
errorValue = zeros(2,steps);                                               % For recording velocity error

% minManipMeasure = 0.1;                                                   % Required for the dampled least squared questions

%% 2.2	Create a trajectory
for i = 1:steps
    x(:,i) = [1.5*cos(deltaTheta*i) + 0.45*cos(deltaTheta*i)
              1.5*sin(deltaTheta*i) + 0.45*cos(deltaTheta*i)];
end

%% 2.3	Create the Transformation Matrix, solve the joint angles
T = [eye(3) [x(:,1);0];zeros(1,3) 1];
qMatrix(1,:) = p2.ikine(T,[0 0],M);

%% 2.4	Use Resolved Motion Rate Control to solve joint velocities 
for i = 1:steps-1
    T = ...;                                                                % End-effector transform at current joint state
    xdot = ...;                                                            % Calculate velocity at discrete time step
    J = ...;                                                               % Get the Jacobian at the current state (use jacob0)
    J = J(1:2,:);                                                           % Take only first 2 rows
    m(:,i)= sqrt(det(J*J'));                                                % Measure of Manipulability
    qdot = ......;                                                          % Solve velocitities via RMRC

%%% 2.7 Use dampled least squared
%     if m(:,i) < minManipMeasure
%         qdot = ...;                                                       %Use dampled least squared
%     else
%         qdot = ...;                                                       % Solve velocitities via RMRC
%     end

    errorValue(:,i) = ...;                                                  % Velocity error
    qMatrix(i+1,:) = ...;                                                   % Update next joint state
    end

%% 2.5	Now plot the trajectory and the error.
figure(1);
set(gcf,'units','normalized','outerposition',[0 0 1 1])
p2.plot(qMatrix,'trail','r-')                                               % Animate the robot
figure(2);
plot(m,'k','LineWidth',1);                                                  % Plot the Manipulability
title('Manipulability of 2-Link Planar')
ylabel('Manipulability')
xlabel('Step')
figure(3)
plot(error','Linewidth',1)
ylabel('Error (m/s)')
xlabel('Step')
legend('x-velocity','y-velocity');

end

