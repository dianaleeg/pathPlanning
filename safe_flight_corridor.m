clear
clc
close all

%% Create Ellipsoid

% function create_elipsoid
% Inputs:
%   * grid: occupnacy grid
%   * x1: 1X2 mat representing the (x, y) position of first node
%   * x2: 1X2 mat representing the (x, y) position of seccond node
% Outputs:
%    * ellispe: container.Map containing center
% 
% function ellipse = get_ellipsoid(grid, x1, x2)
%     ellispe
% 
% end

%% Test
grid = loadMap('city_map.png', 50);
goal = [20, 25];
start = [30, 35];

% goal = [14, 24];
% start = [20, 24];

show(grid)
hold on
scatter(goal(1), goal(2), 'r', 'filled')
scatter(start(1), start(2), 'g', 'filled')

%% Step 1: Set Bounding box
p1 = start;
p2 = goal;

v_max = 10; % m/s
a_max = 9.8; % m/s^2

r_s = v_max^2 / (2 * a_max); % m

theta = atan2(p2(2) - p1(2), p2(1) - p1(1));
R_l_to_w = [[cos(theta), -sin(theta)]; ...
     [sin(theta), cos(theta)]]; % rotation matrix of line segment frame to world frame

R_l_to_w_3 = [[cos(theta), -sin(theta), 0]; ...
     [sin(theta), cos(theta), 0];
     [0, 0, 1]];
 
R_w_to_l = [[cos(theta), sin(theta), 0]; ...
     [-sin(theta), cos(theta), 0];
     [0, 0, 1]]; % rotation matrix of world frame to line segment frame
 
bb_world = []; % Bounding box points in world frame
bb_occupancy = [];
for i = -r_s:r_s
    for j = -r_s:distance(p1, p2) + r_s
        bb_world = [bb_world; p1 + (R_l_to_w * [j; i])'];
    end
end
scatter(bb_world(:,1), bb_world(:,2), 'b');

%% Step 2: Calculate Ellipse 
bb_occupancy = getOccupancy(grid, bb_world);

if ~any(bb_occupancy(:) == 1)
    % TODO no obstacles case
end

% Ellipse midpoint
midpoint = mean([p1; p2]);
scatter(midpoint(1), midpoint(2), 'black');

% Ellipse length along line's x-axis
len = distance(p1, p2);

% Iterate over bounding box and find closest occupied point
bb_size = size(bb_occupancy);
o_closest_world = [-1, -1]; % Closest obstacle in world frame
o_closest_dist = -1; % Distance to closest obstacle
for i = 1:bb_size(1)
    if bb_occupancy(i)
        o_world = bb_world(i, :);
        if isequal(o_closest_world, [-1, -1]) || dist(midpoint, o_world) < o_closest_dist
            o_closest_world = o_world;
            o_closest_dist = dist(midpoint, o_closest_world);
        end
            
    end
end
scatter(o_closest_world(1), o_closest_world(2), 'white');

% TODO fix this by rotating point to world coordinates then measuring

% Solve for ellipse height
point_to_line = abs(point_to_line_dist(p1, p2, o_closest_world));
o_y = midpoint(2) + point_to_line;
o_x = midpoint(1) + sqrt((point_to_line^2) + (o_closest_dist^2));
height = sqrt(abs(((o_y - midpoint(2))^2) / (1 - (((o_x - midpoint(1))^2) / ((len/2)^2)))));



x = linspace(midpoint(1)-(len/2), midpoint(1)+(len/2));
y_pos = midpoint(2) + sqrt((1 - (((x-midpoint(1)).^2) / ((len/2)^2))) * height);
y_neg = midpoint(2) - sqrt((1 - (((x-midpoint(1)).^2) / ((len/2)^2))) * height);

% x = linspace(-(len/2), (len/2));
% y = sqrt((1 - (((x-midpoint(1)).^2) / ((len/2)^2))) * height);
% 
T = [[1, 0, midpoint(1)];
    [0, 1, midpoint(2)];
    [0, 0, 1]];
T_neg = [[1, 0, -midpoint(1)];
    [0, 1, -midpoint(2)];
    [0, 0, 1]];
ellipse = T * R_l_to_w_3 * T_neg * [x; y_pos; ones(size(x))];
ellipse_neg = T * R_l_to_w_3 * T_neg * [x; y_neg; ones(size(x))];


plot(ellipse(1,:), ellipse(2,:))
plot(ellipse_neg(1,:), ellipse_neg(2,:))

% Find line tangent to ellipse at point o
syms x_sym
y = midpoint(2) + sqrt((1 - (((x_sym-midpoint(1))^2) / ((len/2)^2))) * height);
y_dot = diff(y);
slope = vpa(subs(y_dot,x_sym,o_world));

x = linspace(0, ; % Defines the domain as [-15,25] with a refinement of 0.25
x1 = o_closest_world(1); % Specify your starting x
y1 = o_closest_world(2);  % Specify your starting y
y = slope(1)*(x - x1) + y1;
plot(x,y); % plot the graph, and store line reference in a variable.


% Helper Functions
function d = dist(p1, p2)
    d = norm(p1 - p2);
end

% Distance from point (p3) to line (p1, p2)
function d = point_to_line_dist(p1, p2, p3)
      a = [p1, 0] - [p2, 0];
      b = [p3, 0] - [p2, 0];
      d = norm(cross(a,b)) / norm(a);
end
