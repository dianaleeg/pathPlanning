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

% goal = [30, 25];
% start = [20, 35];

% goal = [20, 24];
% start = [14, 24];

% goal = [20, 25];
% start = [25, 30]; 

% goal = [24, 20];
% start = [24, 30];

show(grid)
hold on
scatter(goal(1), goal(2), 'r', 'filled')
scatter(start(1), start(2), 'g', 'filled')

p1 = start;
p2 = goal;

endpoints = SFC(grid, p1,p2)

function endpoints = SFC(grid, p1,p2)
    %% Step 1: Set Bounding box

    v_max = 10; % m/s
    a_max = 9.8; % m/s^2

    r_s = v_max^2 / (2 * a_max); % m

    theta = atan2(p2(2) - p1(2), p2(1) - p1(1));
    R_l_to_w = [[cos(theta), -sin(theta)]; ...
         [sin(theta), cos(theta)]]; % rotation matrix of line segment frame to world frame

    R_l_to_w_3 = [[cos(theta), -sin(theta), 0]; ...
         [sin(theta), cos(theta), 0];
         [0, 0, 1]];

    R_w_to_l_3 = [[cos(theta), sin(theta), 0]; ...
         [-sin(theta), cos(theta), 0];
         [0, 0, 1]]; % rotation matrix of world frame to line segment frame

    bb_world = []; % Bounding box points in world frame
    for i = -r_s:r_s
        for j = -r_s:distance(p1, p2) + r_s
            bb_world = [bb_world; p1 + (R_l_to_w * [j; i])'];
        end
    end
    % scatter(bb_world(:,1), bb_world(:,2), 'b');

    bb_occupancy = getOccupancy(grid, bb_world);
    bb_size = size(bb_occupancy);


    endpoints = []; % Endpoints lines that create SFC polyhedron [x1_1, y1_1, x1_2, y1_2; ]
    %% Step 2: Calculate Ellipse 
    while ismember(1, bb_occupancy)

        for i = 1:bb_size(1)
            bb_point = bb_world(i, :);
            if bb_occupancy(i) == 1
                scatter(bb_point(1), bb_point(2), 'b');
            end
        end

        % Ellipse midpoint
        midpoint = mean([p1; p2]);
        scatter(midpoint(1), midpoint(2), 'black');

        % Ellipse length along line's x-axis
        len = distance(p1, p2);

        % Iterate over bounding box and find closest occupied point

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

        % Solve for ellipse height use pythagorean theorem
        x = [0, grid.XWorldLimits(2)];
        x1 = o_closest_world(1); % Specify your starting x
        y1 = o_closest_world(2);  % Specify your starting y
        T = [[1, 0, x1];
            [0, 1, y1];
            [0, 0, 1]];
        T_neg = [[1, 0, -x1];
                [0, 1, -y1];
                [0, 0, 1]];
        perpendicular = T * R_w_to_l_3 * T_neg * [x; y1 * ones(size(x)); ones(size(x))];
        plot(perpendicular(1,:), perpendicular(2,:))

        [intersect_x, intersect_y] = polyxpoly([p1(1), p2(1)], [p1(2), p2(2)], perpendicular(1,:), perpendicular(2,:));
        point_to_line = dist([intersect_x, intersect_y], o_closest_world);
        % point_to_line = abs(point_to_line_dist(p1, p2, o_closest_world));
        o_y = midpoint(2) + point_to_line;
        o_x = midpoint(1) + sqrt((point_to_line^2) + (o_closest_dist^2));
        height = sqrt(abs(((o_y - midpoint(2))^2) / (1 - (((o_x - midpoint(1))^2) / ((len/2)^2)))));



        % Define ellispe
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
        y = midpoint(2) + sqrt(abs(1 - (((x_sym-midpoint(1))^2) / ((len/2)^2))) * height);
        y_dot = diff(y);
        slope = vpa(subs(y_dot,x_sym,o_world(1)));

    %     x = linspace(0, grid.XWorldLimits(2));
        len_line = abs(grid.XWorldLimits(2) / cos(theta));
        x = -len_line:0.5:len_line;
        x1 = o_closest_world(1); % Specify your starting x
        y1 = o_closest_world(2);  % Specify your starting y
        y = -slope*(x - x1) + y1; % TODO choose positive or negative slope
        T = [[1, 0, x1];
            [0, 1, y1];
            [0, 0, 1]];
        T_neg = [[1, 0, -x1];
            [0, 1, -y1];
            [0, 0, 1]];
        tangent = double(T * R_l_to_w_3 * T_neg * [x; y; ones(size(x))]);
        plot(tangent(1,:),tangent(2,:), 'b'); % plot the graph, and store line reference in a variable.

        x_bounds = [min(bb_world(:, 1)), max(bb_world(:, 1))];
        min_idx = find(round(tangent(1,:)) == round(x_bounds(1)));
        max_idx = find(round(tangent(1,:)) == round(x_bounds(2)));
        
        if (~isempty(min_idx) && ~isempty(max_idx))
            endpoints = [endpoints; tangent(1:2,min_idx(1))', tangent(1:2,max_idx(end))'] % [start_x, start_y, end_x, end_y]
        end
        % plot(x, y,'r')

        % tangent plane to the ellipsoid at this point creates a half space
        for i = 1:bb_size(1)
            if bb_occupancy(i) == 1
                a =1;
            end
            bb_point = bb_world(i, :);
            x = bb_point(1);
            x1 = o_closest_world(1); % Specify your starting x
            y1 = o_closest_world(2);  % Specify your starting y
            y = -slope*(x - x1) + y1; % TODO choose positive or negative slope
            T = [[1, 0, x1];
                [0, 1, y1];
                [0, 0, 1]];
            T_neg = [[1, 0, -x1];
                [0, 1, -y1];
                [0, 0, 1]];
            tangent_point = double(T * R_l_to_w_3 * T_neg * [x; y; ones(size(x))]);
            if round(tangent_point(2)) >= round(midpoint(2)) && round(bb_point(2)) >= round(tangent_point(2))
                bb_occupancy(i) = 0;
            elseif round(tangent_point(2)) <= round(midpoint(2)) && round(bb_point(2)) <= round(tangent_point(2))
                bb_occupancy(i) = 0;
            elseif bb_occupancy(i) == 1
                scatter(bb_point(1), bb_point(2), 'red');
            end
        end
    end
    
    %% Calculate polygons

    intersection_list = tangentLinesIntersections(endpoints)        
    plot(intersection_list(:,1),intersection_list(:,2),'r.','MarkerSize',25)
    
    filter_distance = 5;
    filter_increment = 1;
    continue_incrementing = true;
    
    while (continue_incrementing == true)
        filtered_intersection_list = [];
        for i = 1:length(intersection_list)
            len_to_mp = dist(intersection_list(i,:),midpoint);
            
            if len_to_mp <= filter_distance
                filtered_intersection_list = [filtered_intersection_list; intersection_list(i,:)];
            end
        end
        
        if (isempty(filtered_intersection_list) == 0)
            warning('off', 'MATLAB:polyshape:repairedBySimplify')
            warning('off', 'MATLAB:polyshape:boundary3Points')
            SF_poly = polyshape(filtered_intersection_list);
        
            if (isempty(SF_poly.Vertices) == 0)
                continue_incrementing = false;
                plot(SF_poly)
            end
        end
        filter_distance = filter_distance + filter_increment;
    end
end

%% Helper Functions
function d = dist(p1, p2)
    d = norm(p1 - p2);
end

% Distance from point (p3) to line (p1, p2)
function d = point_to_line_dist(p1, p2, p3)
      a = [p1, 0] - [p2, 0];
      b = [p3, 0] - [p2, 0];
      d = norm(cross(a,b)) / norm(a);
end

function intersection_list = tangentLinesIntersections(endpoints)
    intersection_list = [];
    n = size(endpoints,1);
    indices = nchoosek(1:n,2);
    
    for i = 1:size(indices,1)
        x1 = [endpoints(indices(i,1),1),endpoints(indices(i,1),3)];
        y1 = [endpoints(indices(i,1),2),endpoints(indices(i,1),4)];
        
        x2 = [endpoints(indices(i,2),1),endpoints(indices(i,2),3)];
        y2 = [endpoints(indices(i,2),2),endpoints(indices(i,2),4)];

        p1 = polyfit(x1,y1,1);
        p2 = polyfit(x2,y2,1);
        
        x_intersect = fzero(@(x) polyval(p1-p2,x),3);
        y_intersect = polyval(p1,x_intersect);
        
        if (abs(x_intersect) < 1e4) && (abs(y_intersect) < 1e4)
            intersection_list = [intersection_list; x_intersect y_intersect];
        end
    end
    
    if isempty(intersection_list)
        intersection_list = [endpoints(:,1:2); endpoints(:,3:4)];
    end
end
