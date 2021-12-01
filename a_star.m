close all
clc
clear all

start = [40, 45];
goal = [2, 2];
a_starplanner('city_map.png', start, goal)

function a_starplanner(map_img, start_pos, end_pos)
    
    city_occgrid = loadMap(map_img, 50);
    show(city_occgrid)
    
    costmap = vehicleCostmap(city_occgrid)
    
    
    
    UAVdims = vehicleDimensions(0.5,0.5, 'FrontOverhang',0.1,'RearOverhang',0.1); 
    numCircles = 1;
    inflationRadius = 0.5;
    ccConfig = inflationCollisionChecker(UAVdims, numCircles,'InflationRadius',inflationRadius);
    plot(ccConfig)

    planner = plannerAStarGrid(city_occgrid);
    planner.Map = city_occgrid;

    plan(planner,start_pos, end_pos);
    
    figure(2)
    show(planner)

end