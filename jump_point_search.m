close all
clc
clear all

%% Test

grid = loadMap('city_map.png', 50);
start = [5, 5];
goal = [25, 30];
% load exampleMaps.mat
% grid = binaryOccupancyMap(complexMap);
% start = [2,2];
% goal = [25,2];

figure
show(grid)
hold on
grid on
path = make_path(grid, start, goal, start)


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
function pruned_neighbors = prune_neighbors(grid, x, px, goal)
%     neighbors = getOccupancy(grid, x + 1, "grid");
    pruned_neighbors = zeros([3,3]);
    straight_move = px(1) == x(1) || px(2) == x(2);
    for i = drange(x(1)-1:x(1)+1)
        for j = drange(x(2)-1:x(2)+1)
            n = getOccupancy(grid, [i,j], 'grid');
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
            elseif px == x
                continue;
            % Check if neighbor is goal node
            elseif isequal([i,j], goal)
                continue;
            % Straight Move from px to x
            elseif straight_move
                % Get path lengths
                px_to_n = distance(px, [i, j]);
                px_to_x_to_n = distance(px, x) + distance(x, [i, j]);
                    
                prune =  px_to_n <= px_to_x_to_n;
                
                % Check if path blocked by obstacle
                blockable_h = (i ~= px(1) && abs(j - px(2)) == 2);
                blockable_v = (j ~= px(2) && abs(i - px(1)) == 2);
                
                blocked = (blockable_h && getOccupancy(grid, [i,2], 'grid')) ...
                       || (blockable_v && getOccupancy(grid, [2,j], 'grid'));
                
                if prune && blocked
                    pruned_neighbors(n_i, n_j) = forced();
                elseif prune
                    pruned_neighbors(n_i, n_j) = pruned();
                end
            % Diagonal Move from px to x
            else
                 % Get path lengths
                px_to_n = distance(px, [i, j]);
                px_to_x_to_n = distance(px, x) + distance(x, [i, j]);
                    
                prune =  px_to_n < px_to_x_to_n;
                
                % Check if path blocked by obstaclde
                blockable_h = (i == px(1) && abs(j - px(2)) == 2);
                blockable_v = (j == px(2) && abs(i - px(1)) == 2); %TODO check this
                
                blocked = (blockable_h && getOccupancy(grid, [i,2], 'grid')) ...
                       || (blockable_v && getOccupancy(grid, [2,j], 'grid'));
                
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
function out = jump(grid, x, d, g, s)
    n = step_node(x, d);
    
    % Check if n is within grid or is an obstacle
    if n(1) < grid.XLocalLimits(1) || ...
       n(2) < grid.YLocalLimits(1) || ...
       n(1) > grid.XLocalLimits(2) || ...
       n(2) > grid.YLocalLimits(2) || ...
       getOccupancy(grid, n, 'grid') == occupied()
%         fprintf('null \n')
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
        jump_diag = jump(grid, n, di, g, s);
        if ~isequal(jump_diag, null())
            out = n;
            return
        end
        di = [0, d(2)];
%         fprintf('diagnol special jump 2\n')
        jump_diag = jump(grid, n, di, g, s);
        if ~isequal(jump_diag, null())
            out = n;
            return
        end
            
%         % Check all directions 2 steps away
%         for i = drange(-1:1)
%             for j = drange(-1:1)
%                 di = [i, j];
%                 % Don't check n or x
%                 if isequal(di, [0, 0]) || isequal(x, step_node(n, di)) 
%                     continue
%                 end
%                 jump1 = jump(grid, n, di, g, s);
%                 if ~isequal(jump1, null()) 
%                     fprintf('diagnol special jump\n')
%                     jump2 = jump(grid, jump1, di, g, s);
%                     if ~isequal(jump2, null())
%                         out = n;
%                         fprintf('diagnol special \n')
%                         return
%                     end
%                 end
%             end
%         end
    end
    % else jump recursively
%     fprintf('recursive \n')
    out = jump(grid, n, d, g, s);
end

%% Identify Successors
% Inputs:
%   * grid: Occupnacy grid
%   * x: 1x2 mat representing current node (x, y)
%   * s: 1x2 mat representing start node (x, y)
%   * g: 1x2 mat representing goal node (x, y)
% Outputs:
%   * successors of x

function successors = identify_successors(grid, x, g, s)
    successors = [];
    neighbors = prune_neighbors(grid, x, x, g);
    for i = drange(1:3)
        for j = drange(1:3)
            d = [i-2, j-2];
            if d(1) == 0 && d(2) == 0
                continue
            elseif neighbors(i, j) == unoccupied() || neighbors(i, j) == forced()
%                 fprintf(' jumping \n')
                n = jump(grid, x, d, g, s);
                if ~isequal(n, null())
                    successors = [successors; n];
                end
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

function path = make_path(grid, x, g, s)
    successors = identify_successors(grid, x, g, s);
    
    plot(x(1),x(2),'r.','MarkerSize',15)
    drawnow
    for i = 1:size(successors,1)
        successor = successors(i, :)
        path = make_path(grid, successor, g, s);
        has_goal =intersect(path, g,'rows');
        if ~isempty(has_goal)
            path = [path; successors(i, :)];
            return
        end
    end
    path = null();
end