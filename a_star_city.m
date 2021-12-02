%% Path Planning for Drone
% Move drone using A* Planner
%% 
% Load map of city 

clear all
close all
clc

% load image and create Occupancy map from image
[city_occgrid, city_occgrid_unscaled] = loadMap('city_map.png', 100);
show(city_occgrid_unscaled)

%Set the start and goal states.
start = [5,5,0];
goal = [50,60,0];

%%
UAVsize = 0.2;
inflated_city_occgrid = inflateMap(city_occgrid, UAVsize, 1.0);
[time_traveled, dist_traveled, pthObj, solnInfo] = astar(inflated_city_occgrid, start, goal);

if not(isfolder('figures'))
    mkdir('figures')
end
    
plotSolvedPath(city_occgrid_unscaled,solnInfo,pthObj,'A* - City Occupancy Grid with Path','/figures/a_star_city_path_1.png');