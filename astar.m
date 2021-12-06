function [new_pthObj, solnInfo] = astar(map, start_pos, end_pos)
    
    %remap start and goal
    new_start_pos(1) = map.GridSize(2) - start_pos(2);
    new_start_pos(2) = start_pos(1);
    new_end_pos(1) = map.GridSize(2) - end_pos(2);
    new_end_pos(2) = end_pos(1);

    %Create the path planner and increase max connection distance.
    planner = plannerAStarGrid(map);

    %Plan a path with default settings.
    rng(100,'twister'); % for repeatable result
    
    %call planner
    [pthObj,solnInfo] = plan(planner,new_start_pos,new_end_pos);
    
    new_pthObj(:,1) = pthObj(:,2);
    new_pthObj(:,2) = map.GridSize(2) - pthObj(:,1);

end