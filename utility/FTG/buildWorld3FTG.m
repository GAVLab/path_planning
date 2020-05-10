function [world] = buildWorld2FTG()
%BUILDWORLD2 Summary of this function goes here
%   Detailed explanation goes here

import simulation.*

% Build map
gridResolution = 2; % pixels per meter
gridHeight = 100;
gridWidth = 100;


%% Build dumb bicycle

clear initialState;
initialState.position = [10; 10];
initialState.orientation = pi/8;

initialState.lonVel = 10;
initialState.latVel = 0;
initialState.yawRate = 0;

initialState.velocity(1) = ... 
    initialState.lonVel * cos(initialState.orientation) + ...
    initialState.latVel * sin(initialState.orientation);
initialState.velocity(2) = ...
    initialState.lonVel * sin(initialState.orientation) - ...
    initialState.latVel * cos(initialState.orientation);
initialState.steerAngle = 0;

dumbBicycle1 = simulation.actors.Bicycle2Dof(initialState, 2);

disp("Actor: 2DOF Bicycle")
disp("Location: (3,3)");

%% Build dumb bicycle
clear initialState;
initialState.position = [3,3];
initialState.orientation = pi/4;

initialState.lonVel = 10;
initialState.latVel = 0;
initialState.yawRate = 0;

initialState.velocity(1) = ... 
    initialState.lonVel * cos(initialState.orientation) + ...
    initialState.latVel * sin(initialState.orientation);
initialState.velocity(2) = ...
    initialState.lonVel * sin(initialState.orientation) - ...
    initialState.latVel * cos(initialState.orientation);
initialState.steerAngle = 0;

dumbBicycle2 = simulation.actors.Bicycle2Dof(initialState, 3);

disp("Actor: 2DOF Bicycle")
disp("Location: (3,3)");
%% Build A* Bicycle
clear initialState;
initialState.position = [1; 1];
initialState.orientation = pi/4;

initialState.lonVel = 10;
initialState.latVel = 0;
initialState.yawRate = 0;

initialState.velocity(1) = ... 
    initialState.lonVel * cos(initialState.orientation) + ...
    initialState.latVel * sin(initialState.orientation);
initialState.velocity(2) = ...
    initialState.lonVel * sin(initialState.orientation) - ...
    initialState.latVel * cos(initialState.orientation);

goal = [35; 25];

ftgBicycle = simulation.actors.FTGBicycle2Dof(initialState, 1, goal);

disp("Actor: A* Bicycle")
disp("Location: (1,1)")

%% Build world
world = simulation.World(gridWidth, gridHeight, gridResolution, {dumbBicycle1, dumbBicycle2, ftgBicycle});

end

