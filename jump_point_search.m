clc
clear
% Load Occupancy Grid
load exampleMaps.mat
map = binaryOccupancyMap(simpleMap,2);
show(map)

% Function distance
% Inputs:
%   * p1: point 1 [x, y]
%   * p2: point 2 [x, y]
% Output:
%   Straight line distance between p1 and p2
function dist = distance(p1, p2)
    dist = sqrt(((p1(1) - p2(1))^2) + ((p1(2) - p2(2))^2));
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
function pruned = prune_neighbors(grid, x, px, goal)
%     neighbors = getOccupancy(grid, x + 1, "grid");
    pruned = zeros([3,3]);
    straight_move = px(0) == x(0) || px(1) == x(1);
    for i = drange(x(0)-1:x(0)+1)
        for j = drange(x(1)-1:x(1)+1)
            n = getOccupancy(grid, [i,j], "grid");
            % i, j locations within neighbor matrix 
            n_i = i - x(0) + 1;
            n_j = j - x(1) + 1;
            
            % Skip x
            if isequal([i,j], x)
                continue
            % If neighbor is occupied continue
            elseif n == occupied()
                pruned(n_i,n_j) = occupied();
            elseif isequal([i,j], goal)
                continue; 
            % Straight Move from px to x
            elseif staight_move
                % Get path lengths
                px_to_n = distance(px, [i, j]);
                px_to_x_to_n = distance(px, x) + distance(x, [i, j]);
                    
                prune =  px_to_n <= px_to_x_to_n;
                
                % Check if path blocked by obstacle
                blockable_h = (i ~= px(1) && abs(j - px(2)) == 2);
                blockable_v = (j ~= px(2) && abs(i - px(1)) == 2);
                
                blocked = (blockable_h && getOccupancy(grid, [i,2], "grid")) ...
                       || (blockable_v && getOccupancy(grid, [2,j], "grid"));
                
                if prune && blocked
                    pruned(n_i, n_j) = forced();
                elseif prune
                    pruned(n_i, n_j) = pruned();
                end
            % Diagonal Move from px to x
            else
                 % Get path lengths
                px_to_n = distance(px, [i, j]);
                px_to_x_to_n = distance(px, x) + distance(x, [i, j]);
                    
                prune =  px_to_n < px_to_x_to_n;
                
                % Check if path blocked by obstaclde
                blockable_h = (i == px(1) && abs(j - px(2)) == 2);
                blockable_v = (j == px(2) && abs(i - px(1)) == 2);
                
                blocked = (blockable_h && getOccupancy(grid, [i,2], "grid")) ...
                       || (blockable_h && getOccupancy(grid, [2,j], "grid"));
                
                if prune && blocked
                    pruned(n_i, n_j) = forced();
                elseif prune
                    pruned(n_i, n_j) = pruned();
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
%   * n: 1x2 mat prepresenting the (x, y) of the new mat
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
function out = jump(grid, x, d, g, s)
    n = step_node(x, d);
    % Check if n is within grid or is an obstacle
    if n(0) < 0 || n(1) < 0 || ...
       n(0) > grid.GridSize(0) || ...
       n(1) > grid.GridSize(1) ||
       getOccupancy(grid, n, "grid");
       
        out = null();
        return
    % Check if n is the goal node
    elseif isequal(n, g)
        out = n;
        return
    % Check for forced neighbor 
    elseif ismember(forced(), prune_neighbors(grid, n, x, g))
        out = n;
        return
    % Check for diagonal condition (Condition 3 in Harabor Paper)
    elseif d(1) ~= 0 && d(2) ~= 0
        % Check all directions 2 steps away
        for i = drange(-1:1)
            for j = drange(-1:1)
                di = [i, j];
                % Don't check n or x
                if isequal(di, [0, 0]) || isequal(x, step_node(n, di)) 
                    continue
                end
                jump1 = jump(grid, n, di, g, s);
                if ~isequal(jump1, null())  
                    jump2 = jump(grid, jump1, di, g, s);
                    if ~isequal(jump2, null())
                        out = n;
                        return
                    end
                end
            end
        end
    % else jump recursively
    out = jump(n, d, s, g);
end


%% Identify Successors
% Inputs:
%   * grid: Occupnacy grid
%   * x: 1x2 mat representing current node (x, y)
%   * s: 1x2 mat representing start node (x, y)
%   * g: 1x2 mat representing goal node (x, y)
% Outputs:
%   * successors of x