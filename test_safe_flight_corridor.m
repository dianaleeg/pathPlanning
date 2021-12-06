close all
clear
clc

%% city map

[occgrid, occgrid_unscaled]= loadMap('city_map_low_res.png', 50);
map = occgrid_unscaled;

% works #1
start = [25, 30];
goal = [7, 12];

% loops through obstacle
% start = [40, 30];
% goal = [7, 12];

% goes through wall
% start = [2, 2];
% goal = [20, 24];

% start = [25, 30];
% goal = [38, 16];

[occgrid, occgrid_unscaled]= loadMap('city_map.png', 50);
figure
show(occgrid_unscaled)
hold on
grid on

%% house map
% 
% [occgrid, occgrid_unscaled]= loadMap('house_map.png', 50);
% map = inflateMap(occgrid_unscaled,10,1.25);
% 
% % works #1
% start = [40, 25];
% goal = [25, 15];
% 
% figure
% show(map)
% hold on
% grid on


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

%path = [25,30;13,18;11,16;7,12];

path_size = size(path);
for i = 1:path_size(1)-1
    SF_poly{i} = safe_flight_corridor(map, path(i,:), path(i+1,:));
end

[min_path, min_path_length] = SFC_trajGen(path(1,:), path(end,:), SF_poly)
%%

%plotSolvedPath(map, [], min_path, 'JPS/SFC - City Occupancy Grid with Path','/figures/JPS_SFC_city_path_4.png')