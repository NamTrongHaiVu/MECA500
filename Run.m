%% Robotics
% Lab 4 - Question 3 & 4 - Inverse Kinematics & Joint Interpolation
function RunDobot

clear all; close all; clc
%%
interpolation = 2;                                                          % 1 = Quintic Polynomial, 2 = Trapezoidal Velocity
steps = 10;                                                                % Specify no. of steps

robot = LinearMYUR3(false);                                                         


%% Define End-Effector transformation, use inverse kinematics to get joint angles
q1 =  [0, 0, 0, 0];
q2 =  [0.9113, -0.5942, -0.01781, 0]  ;                                                   % Use inverse kinematics to get the joint angles

%% Interpolate joint angles, also calculate relative velocity, accleration
qMatrix = jtraj(q1,q2,steps);
switch interpolation
    case 1
        qMatrix = jtraj(q1,q2,steps);
    case 2
        s = lspb(0,1,steps);                                             	% First, create the scalar function
        qMatrix = nan(steps,4);                                             % Create memory allocation for variables
            for i = 1:steps
                qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;                   	% Generate interpolated joint angles
            end
    otherwise
        error('interpolation = 1 for Quintic Polynomial, or 2 for Trapezoidal Velocity')
end
        
velocity = zeros(steps,4);
acceleration  = zeros(steps,4);
for i = 2:steps
    velocity(i,:) = qMatrix(i,:) - qMatrix(i-1,:)                          % Evaluate relative joint velocity
    acceleration(i,:) = velocity(i,:) - velocity(i-1,:);                    % Evaluate relative acceleration
end

%% Plot the results
figure(1)

for i =1:steps
    pause(0.1);
    robot.my3.animate(qMatrix(i,:))                                             % Plot the motion between poses, draw a red line of the end-effector path
end

%% test
