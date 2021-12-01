close all
clc
clear all

start = [40, 45];
goal = [2, 2];
a_starplanner("city_map.png", start, goal)

function a_starplanner(map_img, start_pos, end_pos)
    
    grid = loadMap(map_img, 50);
    planner = plannerAStarGrid(grid);
    planner.Map = grid;

    plan(planner,start_pos, end_pos);
    
    show(planner)

end