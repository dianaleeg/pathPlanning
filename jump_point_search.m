close all
clc
clear all

%% Test

% [scaled_grid, grid] = loadMap('city_map.png', 50);
% start = [30, 35];
% goal = [5, 5];

[scaled_grid, grid] = loadMap('house_map.png', 50);
start = [35,20];
goal = [5,5];

path = [];

figure
show(grid)
hold on
grid on

plot(start(1),start(2),'g.','MarkerSize',15)
drawnow

nodes_visited = 0;
[path, nodes_visited] = make_path(grid, start, goal, start, nodes_visited)

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
            elseif px == x
                continue;
            % Check if neighbor is goal node
            elseif isequal([i,j], goal)
                continue;
            % Straight Move from px to x
            elseif straight_move
                % Get path lengths
                px_to_n = dist(px, [i, j]);
                px_to_x_to_n = dist(px, x) + dist(x, [i, j]);
                    
                prune =  px_to_n <= px_to_x_to_n;
                
                % Check if path blocked by obstacle
                blockable_h = (i ~= px(1) && abs(j - px(2)) == 2);
                blockable_v = (j ~= px(2) && abs(i - px(1)) == 2);
                
                blocked = (blockable_h && getOccupancy(grid, [i,2])) ...
                       || (blockable_v && getOccupancy(grid, [2,j]));
                
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
                blockable_h = (i == px(1) && abs(j - px(2)) == 2);
                blockable_v = (j == px(2) && abs(i - px(1)) == 2); %TODO check this
                
                blocked = (blockable_h && getOccupancy(grid, [i,2])) ...
                       || (blockable_v && getOccupancy(grid, [2,j]));
                
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
            
%         % Check all directions 2 steps away
%         for i = drange(-1:1)
%             for j = drange(-1:1)
%                 di = [i, j];
%                 % Don't check n or x
%                 if isequal(di, [0, 0]) || isequal(x, step_node(n, di)) 
%                     continue
%                 end
%                 [jump1, nodes_visited] = jump(grid, n, di, g, s, nodes_visited+1);
%                 if ~isequal(jump1, null()) 
%                     fprintf('diagnol special jump\n')
%                     [jump2, nodes_visited] = jump(grid, jump1, di, g, s, nodes_visited+1);
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
    neighbors = prune_neighbors(grid, x, x, g);
    for i = drange(1:3)
        for j = drange(1:3)
            d = [i-2, j-2];
            if d(1) == 0 && d(2) == 0
                continue
            elseif sign(s(1) - x(1)) == d(1) && sign(s(2) - x(2)) == d(2)
                continue
            elseif neighbors(i, j) == unoccupied() || neighbors(i, j) == forced()
%                 fprintf(' jumping \n')
                [n, nodes_visited] = jump(grid, x, d, g, s, nodes_visited+1);
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

function [path, nodes_visited] = make_path(grid, x, g, s, nodes_visited)
    global path
    [successors, nodes_visited] = identify_successors(grid, x, g, s, nodes_visited);
       
    found_successor = false;
    i = 1;

    %for i = 1:size(successors,1)
    while found_successor == false && i <= size(successors,1)
        if ~isempty(path)
            c = ismember(path,successors(i,:),'row');
        else
            path = s;
            c = 0;
        end
        
        if ~any(c(:))
            successor = successors(i,:);
            found_successor = true;
        else
            successor = successors(1,:);
        end
        
        i = i+1;
        path = [path; successor]
        
        plot(successor(1),successor(2),'g.','MarkerSize',15)
        drawnow

        if (all(successor ~= x)) 
            [path, nodes_visited] = make_path(grid, successor, g, x, nodes_visited);
        end
    end
    
    if (path(end,:) ~= g)
        warning('Goal not achievable! Stopping...')
    end
end
