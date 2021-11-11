
%statespace
ss = stateSpaceSE2;

%statespace validaror
sv = validatorOccupancyMap(ss);

%load example map
load exampleMaps
map = occupancyMap(simpleMap,10);
sv.Map = map;

%some validation distance threshold
sv.ValidationDistance = 0.01;

%Update state space bounds to be the same as map limits.
ss.StateBounds = [map.XWorldLimits;map.YWorldLimits; [-pi pi]];

%Create the path planner and increase max connection distance.
planner = plannerRRT(ss,sv);
planner.MaxConnectionDistance = 0.3;

%Set the start and goal states.
start = [0.5,0.5,0];
goal = [2.5,0.2,0];

%Plan a path with default settings.
rng(100,'twister'); % for repeatable result
[pthObj,solnInfo] = plan(planner,start,goal);

show(map)
hold on
plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),'.-'); % tree expansion
plot(pthObj.States(:,1),pthObj.States(:,2),'r-','LineWidth',2) % draw path