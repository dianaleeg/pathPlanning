function [min_path, min_path_length] = SFC_trajGen(start, goal, poly_list)
%SFC_TRAJGEN Generates the trajectory for SFC path planning

% figure
% hold on
% grid on
% title('SFC Trajectory Generation Example')
% xlabel('x')
% ylabel('y')
% plot(start(1),start(2),'r.')
% plot(goal(1),goal(2),'r.')

for i = 1:size(poly_list,2)
    polys{i} = polyshape(poly_list{i}(:,1)',poly_list{i}(:,2)');
    %plot(polys{i});
    if(i>1)
        intersections{i-1} = intersect(polys{i},polys{i-1});
        %plot(intersections{i-1},'FaceColor','green','FaceAlpha',1)
        
        intersect_grids{i-1} = gridPoly(intersections{i-1},2);
        for j = 1:length(intersect_grids{i-1})
            %plot(intersect_grids{i-1}(j,1),intersect_grids{i-1}(j,2),'b.');
        end
    end
end

index = ones(1,size(intersections,2));
incrementing_set = 1;
incrementing = true;
node_count = 1;

while incrementing == true 
    for i = 1:size(intersections,2)
       nodes{i}(node_count,:) = intersect_grids{i}(index(i),:);
       if (index(incrementing_set) == length(intersect_grids{incrementing_set}))
           if (incrementing_set >= size(intersections,2))
               incrementing = false;
           elseif (incrementing_set + 1 <= size(intersections,2))
               index(incrementing_set + 1) = index(incrementing_set + 1) + 1;
               if (index(incrementing_set + 1) == length(intersect_grids{incrementing_set + 1}))
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

for i = 1:length(nodes{1})
    Pos = start;
    for j = 1:size(intersections,2)
        Pos(j+1,:) = nodes{j}(i,:); 
    end
    Pos(end+1,:) = goal;

    T = 0:1:size(Pos,1)-1; % Time vector

    [Ps] = MinimumSnapGenerator(T,Pos);
    path = Ps(:,1:end-1)';
    %plot(path(:,1),path(:,2),'r','LineWidth',0.25)
    %drawnow
        
    len = pathLength(path);
    if len < min_path_length
        min_path = path;
        min_path_length = len;
    end
end

%plot(min_path(:,1),min_path(:,2),'b','LineWidth',3)

end

