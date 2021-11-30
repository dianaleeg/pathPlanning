close all
clc
clear all

start = [40, 45];
goal = [2, 2];
a_starplanner("city_map.png", start, goal)

function a_starplanner(map_img, start_pos, end_pos)
    
    %map_img = imread('city_map.png');
    grid = loadMap('city_map.png', 50);
    planner = plannerAStarGrid(grid);

    plan(planner,start_pos, end_pos);
    
    show(planner)

end