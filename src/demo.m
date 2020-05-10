%{

%}
clc;clear all;close all
addpath('utility/AStar')
addpath('utility/FTG')

import simulation.*;

%{
import simulation.World;
import simulation.actors.AbstractActor;
import simulation.actors.DummyActor;
import simulation.actors.Bicycle2Dof;
import simulation.actors.FTGBicycle2Dof;
%}

%% Build world

% A*
%world = buildWorld1AStar(); % Head on
%world = buildWorld2AStar(); % Intersecting paths
world = buildWorld3AStar(); % Multiple oscillating cars alongside

% FTG
%world = buildWorld1FTG(); % Head on
%world = buildWorld2FTG(); % Intersecting paths
%world = buildWorld3FTG(); % Multiple oscillating cars alongside

%{
NOTE: THE ACTOR WE CARE ABOUT WILL ALWAYS HAVE ID 1. IT WILL ALWAYS HAVE A
GOAL.
%}

%% Run

dt = 0.01;

for i = 1 : 800
    world.step(dt);
    mainActor = world.getActorById(1);
    if mainActor.isCollided()
        break;
    end
    if mainActor.isGoalReached()
        break;
    end
    world.time_
end

figure(2)
hold on

actors = world.actors_;
lostActors = world.lostActors_;

figure(2)
plot(mainActor.history_(2,:), mainActor.history_(3,:), 'go');
hold on;
plot(mainActor.goal_(1), mainActor.goal_(2), 'gx');

id = mainActor.id_;
actors(id) = [];

for i = 1 : length(actors)
    if isempty(actors{i})
        continue
    end
    actor = actors{i};
    
    hist = actor.history_;
    plot(hist(2,:), hist(3,:), 'o');
    hold on
end

for i = 1 : length(world.lostActors_)
    actor = world.lostActors_{i};
    hist = actor.history_;
    plot(hist(2,:), hist(3,:), 'bx');
end
hold on
plot(mainActor.filter.remStates{1,2}.xGlobal,mainActor.filter.remStates{1,2}.yGlobal,'*')
hold on
plot(mainActor.filter.remStates{1,3}.xGlobal,mainActor.filter.remStates{1,3}.yGlobal,'*')
