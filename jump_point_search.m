close all
clc
clear

%% Test

grid = loadMap('city_map.png', 50);

% goal = [20, 25];
% start = [30, 35];

goal = [33, 40];
start = [20, 35];

% start = [450, 450];
% goal = [100, 100];

% load exampleMaps.mat
% grid = binaryOccupancyMap(complexMap);
% start = [25,35];
% goal = [2,2];

% path = [];

figure
show(grid)
hold on
grid on

plot(start(1),start(2),'g.','MarkerSize',15)
plot(goal(1),goal(2),'r.','MarkerSize',15)
drawnow

nodes_visited = 0;
nodes = [];
[path, nodes_visited, nodes] = make_path(grid, start, start, goal, start, nodes_visited, nodes)

plot(path(:,1), path(:,2), 'y.', 'MarkerSize',15)
plot(start(1),start(2),'r.','MarkerSize',15)
plot(goal(1),goal(2),'r.','MarkerSize',15)
drawnow

% Function dist
% Inputs:
%   * p1: point 1 [x, y]
%   * p2: point 2 [x, y]
% Output:
%   Straight line distance between p1 and p2
function d = dist(p1, p2)
    d = sqrt(((p1(1) - p2(1))^2) + ((p1(2) - p2(2))^2));
end

%% Pruning Rules
% x: node
% p(x): parent of x
% 
% pi: Path from p(x) to n with x in between
% pi': Path from p(x) to n without x

% Straight moves:
%   Prune all neighbors where len({p(x), n} / x) <= len({p(x), x, n})
% Diagonal moves:
%   Prune all neighbors where len({p(x), n} / x) < len({p(x), x, n})

% Define Constants
% TODO: Do this better
function out = unoccupied()
    out = 0;
end
function out = occupied()
    out = 1;
end
function out = pruned()
    out = -1;
end
function out = forced()
    out = 2;
end
function out = null()
    out = [-1, -1];
end


% function prune_neighbors
% Inputs:
%   * grid: occupnacy grid
%   * x: 1X2 mat representing the (x, y) position of the current node
%   * px: 1x2 mat representing the parent of x (x,y)
%   * goal: 1x2 mat representing the goal node (x,y)
% Outputs:
%    * 3x3 Mat representing pruned neighbors of x
function pruned_neighbors = prune_neighbors(grid, x, px, goal)
%     neighbors = getOccupancy(grid, x + 1, "grid");
    pruned_neighbors = zeros([3,3]);
    straight_move = px(1) == x(1) || px(2) == x(2);
    for i = drange(x(1)-1:x(1)+1)
        for j = drange(x(2)-1:x(2)+1)
            n = getOccupancy(grid, [i,j]);
            % i, j locations within neighbor matrix 
            n_i = i - x(1) + 2;
            n_j = j - x(2) + 2;
            
            % Skip x
            if isequal([i,j], x)
                continue
            % If neighbor is occupied continue
            elseif n == occupied()
                pruned_neighbors(n_i,n_j) = occupied();
            % Check if px == x (first move)
            elseif isequal(px, x)
                continue;
            % Check if neighbor is goal node 
            elseif isequal([i,j], goal) 
                continue;
            % Check if neighbor is parent node
            elseif isequal([i,j], px)
                pruned_neighbors(n_i, n_j) = pruned();
            % Straight Move from px to x
            elseif straight_move
                % Get path lengths
                px_to_n = dist(px, [i, j]);
                px_to_x_to_n = dist(px, x) + dist(x, [i, j]);
                    
                prune =  px_to_n <= px_to_x_to_n;
                
                % Check if path blocked by obstacle
                blockable_h = px(2) == x(2) && j ~= px(2) && i ~= px(1) && j ~= x(1);
                blockable_v = px(1) == x(1) && i ~= px(1) && j ~= px(2) && j ~= x(2);
                
                blocked = (blockable_h && getOccupancy(grid, [x(1),j])) ...
                       || (blockable_v && getOccupancy(grid, [i,x(2)]));
                
                if prune && blocked
                    pruned_neighbors(n_i, n_j) = forced();
                elseif prune
                    pruned_neighbors(n_i, n_j) = pruned();
                end
            % Diagonal Move from px to x
            else
                 % Get path lengths
                px_to_n = dist(px, [i, j]);
                px_to_x_to_n = dist(px, x) + dist(x, [i, j]);
                    
                prune =  px_to_n < px_to_x_to_n;
                
                % Check if path blocked by obstaclde
                blockable_h = (i ~= px(1) && abs(i - px(1)) == 2) && j == px(2);
                blockable_v = (j ~= px(2) && abs(j - px(2)) == 2) && i == px(1);
                
                blocked = (blockable_h && getOccupancy(grid, [px(1),j])) ...
                       || (blockable_v && getOccupancy(grid, [i,px(2)]));
                
                if prune && blocked
                    pruned_neighbors(n_i, n_j) = forced();
                elseif prune
                    pruned_neighbors(n_i, n_j) = pruned();
                end
            end
        end
    end
end
%% Step
% Inputs:
%   * x: 1X2 mat representing the (x, y) position of the current node
%   * d: 1x2 mat representing direction of travel (dx,dy) (can be -1, 0, 1)
% Outputs:
%   * n: 1x2 mat prepresenting the (x, y) of the new node
function n = step_node(x, d)
    n = x + d;
end
%% Jump
% Inputs:
%   * grid: Occupnacy grid
%   * x: 1X2 mat representing the (x, y) position of the current node
%   * d: 1x2 mat representing direction of travel (dx,dy) (can be -1, 0, 1)
%   * g: 1x2 mat representing the goal node (x,y)
%   * s: 1x2 mat representing the start node (x,y)
% Outputs:
%   
function [out, nodes_visited] = jump(grid, x, d, g, s, nodes_visited)
    n = step_node(x, d);
    plot(n(1),n(2),'b.','MarkerSize',5)
    %drawnow
    % Check if n is within grid or is an obstacle
    if n(1) <= grid.XLocalLimits(1) || ...
       n(2) <= grid.YLocalLimits(1) || ...
       n(1) >= grid.XLocalLimits(2) || ...
       n(2) >= grid.YLocalLimits(2) || ...
       getOccupancy(grid, n) == occupied()
%         fprintf('null \n')
        
        plot(n(1),n(2),'r.','MarkerSize',10)
        %drawnow
        out = null();
        
        return
    % Check if n is the goal node
    elseif isequal(n, g)
        out = n;
%         fprintf('n == goal \n')
        return
    % Check for forced neighbor 
    elseif ismember(forced(), prune_neighbors(grid, n, x, g))
        out = n;
%         fprintf('forced \n')
        return
    % Check for diagonal condition (Condition 3 in Harabor Paper)
    elseif d(1) ~= 0 && d(2) ~= 0
        di = [d(1), 0];
%         fprintf('diagnol special jump\n')
        [jump_diag, nodes_visited] = jump(grid, n, di, g, s, nodes_visited+1);
        if ~isequal(jump_diag, null())
            out = n;
            return
        end
        di = [0, d(2)];
%         fprintf('diagnol special jump 2\n')
        [jump_diag, nodes_visited] = jump(grid, n, di, g, s, nodes_visited+1);
        if ~isequal(jump_diag, null())
            out = n;
            return
        end
    end
    % else jump recursively
    fprintf('recursive \n')
    [out, nodes_visited] = jump(grid, n, d, g, s, nodes_visited+1);
end

%% Identify Successors
% Inputs:
%   * grid: Occupnacy grid
%   * x: 1x2 mat representing current node (x, y)
%   * s: 1x2 mat representing start node (x, y)
%   * g: 1x2 mat representing goal node (x, y)
% Outputs:
%   * successors of x

function [successors, nodes_visited] = identify_successors(grid, x, g, s, nodes_visited)
    successors = [];
    if g(1) > s(1)
        i_range = [3, 2, 1];
    else
        i_range = [1, 2, 3];
    end
    if g(2) > s(2)
        j_range = [3, 2, 1];
    else
        j_range = [1, 2, 3];
    end
    for i = i_range
        for j = j_range
            d = [i-2, j-2];
            if isequal(d, [0, 0])
                continue
%             elseif ~isequal(d, [1, 0]) % TODO: DEBUG ONLY REMOVE
%                 continue
            end
            [jp, nodes_visited] = jump(grid, x, d, g, s, nodes_visited);
            if ~isequal(jp, null())
                successors = [successors; jp];
            end
                
        end
    end
end

%% Make Path

% Inputs:
%   * grid: Occupnacy grid
%   * x: 1x2 mat representing current node (x, y)
%   * s: 1x2 mat representing start node (x, y)
%   * g: 1x2 mat representing goal node (x, y)
% Outputs:
%   * successors of x

function [path, nodes_visited, nodes] = make_path(grid, px, x, g, s, nodes_visited, nodes)
    x
    plot(x(1), x(2), 'c.', 'MarkerSize',15);
    drawnow
    nodes = [nodes; x];
    if isequal(x, g)
       path = g;
       return
    end
    [successors, nodes_visited] = identify_successors(grid, x, g, s, nodes_visited);
    successors
    
    if ~isequal(successors, [])
        scatter(successors(:,1), successors(:,2), 'g');
        for i = 1:size(successors(:,1))
            if ~isequal(nodes, []) && ismember(successors(i, :), nodes, 'rows')
                continue
            end
            [path, nodes_visited, nodes] = make_path(grid, x, successors(i,:), g, s, nodes_visited, nodes);
            if ~isequal(path, null())
                [tf, index]=ismember(g, path,'rows');
                if tf
                    path = [path; x];
                    return
                end 
            end
        end
    end
    path = null(); 
%     global path
%     [successors, nodes_visited] = identify_successors(grid, x, g, s, nodes_visited);
%        
%     found_successor = false;
%     i = 1;
% 
%     %for i = 1:size(successors,1)
%     while found_successor == false && i <= size(successors,1)
%         if ~isempty(path)
%             c = ismember(path,successors(i,:),'row');
%         else
%             path = s;
%             c = 0;
%         end
%         
%         if ~any(c(:))
%             successor = successors(i,:);
%             found_successor = true;
%         else
%             successor = successors(1,:);
%         end
%         
%         i = i+1;
%         path = [path; successor]
%         
%         plot(successor(1),successor(2),'g.','MarkerSize',15)
%         drawnow
% 
%         if (all(successor ~= x)) 
%             [path, nodes_visited] = make_path(grid, successor, g, x, nodes_visited);
%         end
%     end
%     
%     if (path(end,:) ~= g)
%         warning('Goal not achievable! Stopping...')
%     end
end
