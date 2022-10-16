clear all; close all; clc;
%% Environment
workspace = [-2 2 -2 2 -0.61 2]; 
environment()

%% Call Robots
r = Meca500;
r2 = UR3;
