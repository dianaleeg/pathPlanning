%% Path Planning for Drone
% Move 4 ft x 4 ft drone using RRT Planner
%% 
% Load map of house 

clear all
close all

% load image and create Occupancy map from image
house_occgrid = loadMap('house_map.png', 50);
show(house_occgrid)
%% 
% Create costmap using vehicleCostmap function from toolbox

costmap = vehicleCostmap(house_occgrid)
%% 
% Modify dimensions of drone and parameters of collision checker.  Drone is 
% 0.5 m x 0.5 m.  
% 
% One circle of radius 0.5 m at the center of the drone will be used to check 
% for collisions.


UAVdims = vehicleDimensions(0.5,0.5, 'FrontOverhang',0.1,'RearOverhang',0.1); 
numCircles = 1;
inflationRadius = 0.5;
ccConfig = inflationCollisionChecker(UAVdims, numCircles,'InflationRadius',inflationRadius);
plot(ccConfig)

costmap.CollisionChecker = ccConfig;

figure
plot(costmap)
%% 
% Run the path planner and plot on the map

%Set the start and goal states.
start = [5,5,0];
goal = [30,40,90];

planner = pathPlannerRRT(costmap);
refPath = plan(planner,start,goal);

plot(planner)
hold on
%%
UAVsize = 0.5;
inflated_house_occgrid = inflateMap(house_occgrid, UAVsize, 1.25)
[time_traveled, dist_traveled, pthObj, solnInfo] = rrt(inflated_house_occgrid, start, goal)

if not(isfolder('figures'))
    mkdir('figures')
end

plotSolvedPath(house_occgrid,solnInfo,pthObj,'House Occupancy Grid with Path','/figures/house_path_1.png');