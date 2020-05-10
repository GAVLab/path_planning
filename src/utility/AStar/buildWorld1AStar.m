function [world] = buildWorld1AStar()

%{
This simulation has a FTG bicycle try to get around an oscillating
obstacle.
%}

import simulation.World;
import simulation.actors.AbstractActor;
import simulation.actors.DummyActor;
import simulation.actors.Bicycle2Dof;
import simulation.actors.FTGBicycle2Dof;

% Build map
gridResolution = 1; % pixels per meter
gridHeight = 100;
gridWidth = 100;


%% Build dumb bicycle
clear initialState;
initialState.position = [1; 11];
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

dumbBicycle1 = simulation.actors.Bicycle2Dof(initialState, 2);

disp("Actor: 2DOF Bicycle")
disp("Location: (3,3)");

clear initialState;
initialState.position = [11; 1];
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

%% Build FTG Bicycle
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
initialState.steerAngle = 0;

goal = [35; 25];

astarBicycle = simulation.actors.AStarBicycle2Dof(initialState, 1, goal);

disp("Actor: A* Bicycle")
disp("Location: (1,1)")

%% Build world
world = simulation.World(gridWidth, gridHeight, gridResolution, {dumbBicycle1, dumbBicycle2, astarBicycle});
%world = simulation.World(gridWidth, gridHeight, gridResolution, {dumbBicycle, ftgBicycle});

end

