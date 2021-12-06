close all
clear
clc

[occgrid, occgrid_unscaled]= loadMap('city_map_low_res.png', 50);
% figure
% show(occgrid_unscaled)
% grid on
map = occgrid_unscaled;
start = [7, 7];
goal = [41, 31];

[occgrid, occgrid_unscaled]= loadMap('city_map.png', 50);
figure
show(occgrid_unscaled)
hold on
grid on
% start = [7, 7];
% goal = [41, 31];
elapsed_time = sfc(map, start, goal)

function elapsed_time = sfc(map, start, goal)

    start_time = clock;
    
    %% Jump Point Search
    nodes = [];
    nodes_visited = 0;

    plot(start(1),start(2),'g.','MarkerSize',15)
    plot(goal(1),goal(2),'r.','MarkerSize',15)
    drawnow

    [path, nodes_visited, nodes] = jump_point_search(map, start, start, goal, start, nodes_visited, nodes);

    plot(path(:,1), path(:,2), 'y.', 'MarkerSize',15)
    plot(start(1),start(2),'r.','MarkerSize',15)
    plot(goal(1),goal(2),'r.','MarkerSize',15)
    drawnow

    %% Safe Flight Corridor
    
    path_size = size(path);
    for i = 1:path_size(1)-1
        SF_poly = safe_flight_corridor(map, path(i,:), path(i+1,:))
    end
    
	[min_path, min_path_length] = SFC_trajGen(nodes(1,:), nodes(end,:), SF_poly)
    elapsed_time = start_time - end_time;
    
	plotSolvedPath(grid, [], min_path, 'JPS/SFC - City Occupancy Grid with Path','/figures/JPS_SFC_city_path_2.png')
end 