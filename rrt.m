
function [time_traveled, pthObj, solnInfo] = rrt(map, start_pos, end_pos)

    %statespace
    ss = stateSpaceSE2;

    %statespace validaror
    sv = validatorOccupancyMap(ss);

    %load map
    sv.Map = map;

    %some validation distance threshold
    sv.ValidationDistance = 0.1;

    %Update state space bounds to be the same as map limits.
    ss.StateBounds = [map.XWorldLimits;map.YWorldLimits; [-pi pi]];
    
    %Create the path planner and increase max connection distance.
    planner = plannerRRT(ss,sv);
    planner.MaxConnectionDistance = 0.3;

    %Plan a path with default settings.
    rng(100,'twister'); % for repeatable result
    
    %start timer after variables are set and map is loaded
    start_time = clock;
    
    %call planner
    [pthObj,solnInfo] = plan(planner,start_pos,end_pos);
    
    %stop timer
    end_time = clock;
    
    time_traveled = end_time - start_time;

end