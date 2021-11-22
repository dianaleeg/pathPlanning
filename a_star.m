function [time_traveled, dist_traveled, map] = a_star(map_img, start_pos, end_pos)

    cost_from_start_to_next_node = 0;
    est_cost_from_next_node_to_last_node= 0;
    calc_cost = cost_from_start_to_next_node + est_cost_from_next_node_to_last_node;
    
    %map_img = imread('city_map.png');
    grid = loadMap('city_map.png', 50);
    start_pos = [1,1];
    end_pos = [10,1];
    
    %start timer after variables are set and map is loaded
    start_time = clock;

    opened = zeros(50,50); % needs to be the list of all nodes to visit
    closed = zeros(50,50); % will populate as nodes are visited
    
    opened(1) = start_pos(1), start_pos(2)
    opened
    
    %start timer once variables are set up and map loaded
    start_time = clock; 
    while size(opened) > 0
        
        %keep running algo
        %next_node = min(opened[]) % open node with least cost
        
    end
    
    %end timer
    end_time = clock;
    
    %take elapsed time
    time_traveled = end_time - start_time

end