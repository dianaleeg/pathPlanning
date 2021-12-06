% The purpose of this code is to implement the Safe Flight Corridors and
% Receding Horizon Path Planning Algorithm
% The authors of this code are: Christian Chang, Christopher Poole, Trevor Rizzo and Diana
% Lee Guzman

close all
clear all
clc

%% Process Overview


%% Environment Setup

% MATLAB Toolboxes Needed:
% 
% Parallel Computing Toolbox
% Automated Driving Toolbox
% Symbolic Math Toolbox

% ensure figures folder exists
if not(isfolder('figures'))
    mkdir('figures')
end

UAVsize = 0.2;

%% Map Generation

[city_occgrid, city_occgrid_unscaled] = loadMap('city_map.png', 50);
figure
show(city_occgrid_unscaled)
title('City Map')
saveas(gcf,[pwd '/figures/city_map.png'])

inflated_city_occgrid = inflateMap(city_occgrid, UAVsize, 1.0);

[house_occgrid, house_occgrid_unscaled] = loadMap('house_map.png', 50);
figure
show(house_occgrid_unscaled)
title('House Map')
saveas(gcf,[pwd '/figures/house_map.png'])

inflated_house_occgrid = inflateMap(house_occgrid, UAVsize, 1.0);

%% Path Planning

start_city = [30,35; 30,35; 30,35];
goal_city = [5,5; 5,5; 5,5];

start_house = [35,20; 35,20; 35,20];
goal_house = [5,5; 5,5; 5,5];

iterations = [1;2;3];
[numRows, numCols] = size(iterations);

%Time Elapsed astar
time_elapsed_astar_grid1 = zeros(numRows,1);
time_elapsed_astar_grid2 = zeros(numRows,1);

%Time Elapsed sfc
time_elapsed_sfc_grid1 = zeros(numRows,1);
time_elapsed_sfc_grid2 = zeros(numRows,1);

%Time elapsed RRT
time_elapsed_rrt_grid1 = zeros(numRows,1);
time_elapsed_rrt_grid2 = zeros(numRows,1);

%Path Distance astar
path_distance_astar_grid1 = zeros(numRows,1);
path_distance_astar_grid2 = zeros(numRows,1);

%Path Distance SFC
path_distance_sfc_grid1 = zeros(numRows,1);
path_distance_sfc_grid2 = zeros(numRows,1);

%Path Distance RRT
path_distance_rrt_grid1 = zeros(numRows,1);
path_distance_rrt_grid2 = zeros(numRows,1);

%Nodes Visited astar
nodes_visited_astar_grid1 = zeros(numRows,1);
nodes_visited_astar_grid2 = zeros(numRows,1);

%Nodes Visited SFC
nodes_visited_sfc_grid1 = zeros(numRows,1);
nodes_visited_sfc_grid2 = zeros(numRows,1);

%Nodes Visited RRT
nodes_visited_rrt_grid1 = zeros(numRows,1);
nodes_visited_rrt_grid2 = zeros(numRows,1);

%Robustness astar
robustness_astar_grid1 = zeros(numRows,1);
robustness_astar_grid2 = zeros(numRows,1);

%Roboustness SFC
robustness_sfc_grid1 = zeros(numRows,1);
robustness_sfc_grid2 = zeros(numRows,1);

%Roboustness RRT
robustness_rrt_grid1 = zeros(numRows,1);
robustness_rrt_grid2 = zeros(numRows,1);

for i = 1:numRows
    
    %RRT
    [time_elapsed_rrt, pthObj, solnInfo] = rrt(inflated_city_occgrid, [start_city(i,1) start_city(i,2) 0], [goal_city(i,1) goal_city(i,2) 0]);   
    mapName = ['RRT - City Occupancy Grid with Path For Iteration ', num2str(i)];
    mapPath = ['/figures/rrt_city_path_', num2str(i),'.png'];
    time_elapsed_rrt_grid1(i,1) = time_elapsed_rrt(6);
    path_distance_rrt_grid1(i,1) = solnInfo.NumNodes;
    [treeRows, treeCols] = size(solnInfo.TreeData);
    nodes_visited_rrt_grid1(i,1) = treeRows;
    robustness_rrt_grid1(i,1) = solnInfo.IsPathFound;
    plotSolvedPath(city_occgrid_unscaled,solnInfo,pthObj,mapName, mapPath);
    
    [time_elapsed_rrt, pthObj, solnInfo] = rrt(inflated_house_occgrid, [start_house(i,1) start_house(i,2) 0], [goal_house(i,1) goal_house(i,2) 90]);   
    mapName = ['RRT - House Occupancy Grid with Path For Iteration ' + num2str(i)];
    mapPath = ['/figures/rrt_house_path_', num2str(i), '.png'];
    time_elapsed_rrt_grid2(i,1) = time_elapsed_rrt(6);
    path_distance_rrt_grid2(i,1) = solnInfo.NumNodes;
    [treeRows, treeCols] = size(solnInfo.TreeData);
    nodes_visited_rrt_grid2(i,1) = treeRows;
    robustness_rrt_grid2(i,1) = solnInfo.IsPathFound;
    plotSolvedPath(city_occgrid_unscaled,solnInfo,pthObj,mapName, mapPath);
    
    %A*
    [time_elapsed_astar, pthObj, solnInfo] = astar(inflated_city_occgrid,  [start_city(i,1) start_city(i,2) 0], [goal_city(i,1) goal_city(i,2) 0]);
    mapName = ['A* - City Occupancy Grid with Path For Iteration ', num2str(i)];
    mapPath = ['/figures/a_star_city_path_',num2str(i), '.png'];
    time_elapsed_astar_grid1(i,1) = time_elapsed_astar(6);
    [treeRows, treeCols] = size(pthObj);
    path_distance_astar_grid1(i,1) = treeRows;
    nodes_visited_astar_grid1(i,1) = solnInfo.NumNodesExplored;
        if(treeRows > 0)
        robustness_astar_grid1(i,1) = 1;
    else 
        robustness_astar_grid1(i,1) = 1;
    end
    plotSolvedPath(city_occgrid_unscaled,solnInfo,pthObj,mapName, mapPath);

    [time_elapsed_astar, pthObj, solnInfo] = astar(inflated_house_occgrid,  [start_house(i,1) start_house(i,2) 0], [goal_house(i,1) goal_house(i,2) 90]);
    time_elapsed_astar_grid2(i,1) = time_elapsed_astar(6);
    [treeRows, treeCols] = size(pthObj);
    path_distance_astar_grid1(i,1) = treeRows;
    nodes_visited_astar_grid1(i,1) = solnInfo.NumNodesExplored;
    if(treeRows > 0)
        robustness_astar_grid2(i,1) = 1;
    else 
        robustness_astar_grid2(i,1) = 1;
    end
    mapName = ['A* - House Occupancy Grid with Path For Iteration ', num2str(i)];
    mapPath = ['/figures/a_star_house_path_', num2str(i), '.png'];
    plotSolvedPath(city_occgrid_unscaled,solnInfo,pthObj, mapName, mapPath);
    
    % JPS & SFC
    [elapsed_time, nodes_visited, grid, min_path] = sfc(inflated_city_occgrid,  [start_city(i,1) start_city(i,2) 0], [goal_city(i,1) goal_city(i,2) 0]);
    mapName = ['SFC - City Occupancy Grid with Path For Iteration ', num2str(i)];
    mapPath = ['/figures/sfc_city_path_',num2str(i), '.png'];
    time_elapsed_sfc_grid1(i,1) = time_elapsed(6);
    %path_distance_sfc_grid1(i,1) = solnInfo.NumNodes;
    %[treeRows, treeCols] = size(solnInfo.TreeData);
    nodes_visited_sfc_grid1(i,1) = nodes_visited;
    %robustness_sfc_grid1(i,1) = solnInfo.IsPathFound;    
    plotSolvedPath(grid,[], min_path, mapName, mapPath);
    
    [elapsed_time, nodes_visited, grid, min_path] = sfc(inflated_house_occgrid, [start_house(i,1) start_house(i,2) 0] , [goal_house(i,1) goal_house(i,2) 90]);
    time_elapsed_sfc_grid2(i,1) = time_elapsed(6);
    %path_distance_sfc_grid2(i,1) = solnInfo.NumNodes;
    %[treeRows, treeCols] = size(solnInfo.TreeData);
    nodes_visited_sfc_grid2(i,1) = nodes_visited;
    %robustness_sfc_grid2(i,1) = solnInfo.IsPathFound;
    mapName = ['SFC - House Occupancy Grid with Path For Iteration ', num2str(i)];
    mapPath = ['/figures/sfc_house_path_', num2str(i), '.png'];
    plotSolvedPath(grid,[], min_path, mapName, mapPath);

end


%% Path Analysis


%% Planner Analysis

%Variables for pass/fail ratio
[pass_rrt, fail_rrt, pass_sfc, fail_sfc] = deal(0, 0, 0, 0);

for i = 1:numRows
    if robustness_rrt_grid1(i,1) == 1
        pass_rrt = pass_rrt + 1;
    else
        fail_rrt = fail_rrt +1;
    end
    if robustness_rrt_grid2(i,1) == 1
        pass_rrt = pass_rrt + 1;
    else
        fail_rrt = fail_rrt +1;
    end
    
    if robustness_sfc_grid1(i,1) == 1
        pass_sfc = pass_sfc + 1;
    else
        fail_sfc = fail_sfc +1;
    end
    if robustness_sfc_grid2(i,1) == 1
        pass_sfc = pass_sfc + 1;
    else
        fail_sfc = fail_sfc +1;
    end
end

%Grab rows & columns
[r1,c1] = size(robustness_sfc_grid1);
[r2,c2] = size(robustness_sfc_grid2);
[r3,c3] = size(robustness_rrt_grid1);
[r4,c4] = size(robustness_rrt_grid2);

%Pass/Fail Ratios
sfc_pass = pass_sfc / (c1 + c2);
sfc_fail = fail_sfc / (c1 + c2);
rrt_pass = pass_rrt / (c1 + c2);
rrt_fail = fail_rrt / (c1 + c2);

%Table
T = table(time_elapsed_sfc_grid1, time_elapsed_rrt_grid1, path_distance_sfc_grid1, path_distance_rrt_grid1, nodes_visited_sfc_grid1, nodes_visited_rrt_grid1, robustness_sfc_grid1, robustness_rrt_grid1)
T = table(time_elapsed_sfc_grid2, time_elapsed_rrt_grid2, path_distance_sfc_grid2, path_distance_rrt_grid2, nodes_visited_sfc_grid2, nodes_visited_rrt_grid2, robustness_sfc_grid2, robustness_rrt_grid2)

%Figure 1: Elapsed Time City Map
scatter(iterations,time_elapsed_rrt_grid1, "black", "filled")
hold on;
scatter(iterations,time_elapsed_sfc_grid1, "red", "filled")
title ("Elapsed Time City Map")
legend('sfc', 'rrt');

hold off;

%Figure 2: Elapsed Time House Map
scatter(iterations,time_elapsed_sfc_grid2, "blue", "filled")
hold on;
scatter(iterations,time_elapsed_rrt_grid2, "cyan", "filled")
title ("Elapsed Time House Map")
legend('sfc', 'rrt');

hold off;

%Figure 3: Path Distance City Map
scatter(iterations,path_distance_sfc_grid1, "black", "filled")
hold on;
scatter(iterations,path_distance_rrt_grid1, "red", "filled")
title ("Path Distance City Map")
legend('sfc', 'rrt');

hold off;

%Figure 4: Path Distance House Map
scatter(iterations,path_distance_sfc_grid2, "blue", "filled")
hold on;
scatter(iterations,path_distance_rrt_grid2, "cyan", "filled")
title ("Path Distance House Map")
legend('sfc', 'rrt');

hold off;

%Figure 5: Nodes Visited City Map
scatter(iterations,nodes_visited_sfc_grid1, "black", "filled")
hold on;
scatter(iterations,nodes_visited_rrt_grid1, "red", "filled")
title ("Nodes Visited City Map")
legend('sfc', 'rrt');

hold off;

%Figure 6: Nodes Visited House Map
scatter(iterations,nodes_visited_sfc_grid2, "blue", "filled")
hold on;
scatter(iterations,nodes_visited_rrt_grid2, "cyan", "filled")
title ("Nodes Visited House Map")
legend('sfc', 'rrt');

hold off;

%Figure 7: Robustness SFC on both maps
labels = {'Pass','Fail'};
pie([sfc_pass, sfc_fail])
title ("SFC Robustness")
legend(labels)

%Figure 7: Robustness RRT on both maps
labels = {'Pass','Fail'};
pie([rrt_pass, rrt_fail])
title ("RRT Robustness")
legend(labels)

%% Custom Functions

