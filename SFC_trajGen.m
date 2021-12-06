function [min_path, min_path_length] = SFC_trajGen(start, goal, polys)
%SFC_TRAJGEN Generates the trajectory for SFC path planning

figure
hold on
grid on
title('SFC Trajectory Generation Example')
xlabel('x')
ylabel('y')
plot(start(1),start(2),'r.')
plot(goal(1),goal(2),'r.')

for i = 1:size(polys,2)
    plot(polys{i});
end

intersections = [];
n = size(polys,2);
indices = nchoosek(1:n,2);
num_intersection = 1;

for i = 1:size(indices,1)
    if ~isempty(polys{indices(i,1)}) && ~isempty(polys{indices(i,2)})
        temp_poly = intersect(polys{indices(i,1)},polys{indices(i,2)});
        if (size(temp_poly.Vertices,1) ~= 0) && (size(temp_poly.Vertices,2) ~= 0)
            intersections{num_intersection} = temp_poly;
            plot(intersections{num_intersection},'FaceColor','green','FaceAlpha',1)
            intersect_grids{num_intersection} = gridPoly(intersections{num_intersection},2);
            
            if (size(intersect_grids{num_intersection},1) == 0 || size(intersect_grids{num_intersection},2) == 0)
                [x_center, y_center] = centroid(intersections{num_intersection}); 
                intersect_grids{num_intersection} = [x_center, y_center; x_center, y_center];
            end
            
            for j = 1:size(intersect_grids{num_intersection},1)
                plot(intersect_grids{num_intersection}(j,1),intersect_grids{num_intersection}(j,2),'b.');
            end

            num_intersection = num_intersection + 1;
        end
    end
end

index = ones(1,size(intersections,2));
incrementing_set = 1;
incrementing = true;
node_count = 1;

while incrementing == true 
    for i = 1:size(intersections,2)
       %if (index(i) < size(intersect_grids{i}))
        nodes{i}(node_count,:) = intersect_grids{i}(index(i),:);
       %end
       if (index(incrementing_set) == size(intersect_grids{incrementing_set},1))
           if (incrementing_set >= size(intersections,2))
               incrementing = false;
           elseif (incrementing_set + 1 <= size(intersections,2))
               index(incrementing_set + 1) = index(incrementing_set + 1) + 1;
               if (index(incrementing_set + 1) == size(intersect_grids{incrementing_set + 1},1))
                   incrementing_set = incrementing_set + 1;
               end
           index(1:incrementing_set) = 1;
           end
       else
           index(incrementing_set) = index(incrementing_set) + 1;
       end
    end
    node_count = node_count + 1;
end

min_path_length = inf;

for i = 1:size(nodes{1},1)
    Pos = start;
    for j = 1:size(intersections,2)
        Pos(j+1,:) = nodes{j}(i,:); 
    end
    Pos(end+1,:) = goal;

    T = 0:1:size(Pos,1)-1; % Time vector

    [Ps] = MinimumSnapGenerator(T,Pos);
    path = Ps(:,1:end-1)';
    plot(path(:,1),path(:,2),'r','LineWidth',0.25)
    drawnow
        
    len = pathLength(path);
    if len < min_path_length
        min_path = path;
        min_path_length = len;
    end
end

plot(min_path(:,1),min_path(:,2),'b','LineWidth',3)

end

