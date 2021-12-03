%% Path Planning for Drone
% Move drone using A* Planner
%% 
% Load map of house 

clear all
close all
clc

% load image and create Occupancy map from image
[house_occgrid, house_occgrid_unscaled] = loadMap('house_map.png', 100);
show(house_occgrid_unscaled)

%Set the start and goal states.
start = [5,5,0];
goal = [55,80,90];

%%
UAVsize = 0.2;
inflated_house_occgrid = inflateMap(house_occgrid, UAVsize, 1.25)
[time_traveled, dist_traveled, pthObj, solnInfo] = astar(inflated_house_occgrid, start, goal)

if not(isfolder('figures'))
    mkdir('figures')
end

plotSolvedPath(house_occgrid_unscaled,solnInfo,pthObj,'A* - House Occupancy Grid with Path','/figures/a_star_house_path_1.png');