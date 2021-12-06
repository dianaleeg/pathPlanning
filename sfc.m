

function [elapsed_time, nodes_visited, grid, min_path] = sfc(map, start, goal)

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
    
    figure
    show(map)
    hold on
    
    path_size = size(path);
    for i = 1:path_size(1)-1
        SF_poly{i} = safe_flight_corridor(map, path(i,:), path(i+1,:));
    end
    
	[min_path, min_path_length] = SFC_trajGen(nodes(1,:), nodes(end,:), SF_poly)
    elapsed_time = start_time - end_time;
    
end 