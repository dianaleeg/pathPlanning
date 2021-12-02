function [time_traveled, dist_traveled, new_pthObj, solnInfo] = astar(map, start_pos, end_pos)
    
    %remap start and goal
    new_start_pos(1) = map.GridSize(2) - start_pos(2);
    new_start_pos(2) = start_pos(1);
    new_end_pos(1) = map.GridSize(2) - end_pos(2);
    new_end_pos(2) = end_pos(1);

    %Create the path planner and increase max connection distance.
    planner = plannerAStarGrid(map);

    %Plan a path with default settings.
    rng(100,'twister'); % for repeatable result
    
    %start timer after variables are set and map is loaded
    start_time = clock;
    
    %call planner
    [pthObj,solnInfo] = plan(planner,new_start_pos,new_end_pos);
    
    %stop timer
    end_time = clock;
    
    new_pthObj(:,1) = pthObj(:,2);
    new_pthObj(:,2) = map.GridSize(2) - pthObj(:,1);
    
    time_traveled = end_time - start_time;
    dist_traveled = 0;

end