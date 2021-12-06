function [elapsed_time, nodes_visited, path, min_path, min_path_length, timeout_occurred] = sfc(map, start, goal, timeout)

    start_time = clock;
    
    %% Jump Point Search
    nodes = [];
    nodes_visited = 0;

    plot(start(1),start(2),'g.','MarkerSize',15)
    plot(goal(1),goal(2),'r.','MarkerSize',15)
    drawnow

    [path, nodes_visited, nodes, timeout_occurred] = jump_point_search(map, start, start, goal, start, nodes_visited, nodes, start_time, timeout);
    
    if(timeout_occurred)
        min_path=[];
        min_path_len = 0;
        elapsed_time = clock - start_time;
        return
    end
    
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
        [SF_poly{i}, timeout_occurred] = safe_flight_corridor(map, path(i,:), path(i+1,:), start_time, timeout);
        if(timeout_occurred)
            min_path=[];
            min_path_len = 0;
            elapsed_time = clock - start_time;
            return
        end
    end
    
    now = clock;
    elapsed = now - start_time;
    if elapsed(6) > timeout
        timeout_occurred = true
        min_path=[];
        min_path_len = 0;
        elapsed_time = clock - start_time;
        return
    end
    
    [min_path, min_path_length] = SFC_trajGen(path(1,:), path(end,:), SF_poly)
    elapsed_time = start_time - end_time;
    
end 