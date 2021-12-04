function path = SFC_trajGen(start, goal, poly_list)
%SFC_TRAJGEN Summary of this function goes here
%   Detailed explanation goes here

% load start, end points

% load polygons

figure
hold on
plot(start(1),start(2),'r.')
plot(goal(1),goal(2),'r.')

for i = 1:size(poly_list,2)
    polys{i} = polyshape(poly_list{i}(:,1)',poly_list{i}(:,2)');
    plot(polys{i});
    if(i>1)
        intersections{i-1} = intersect(polys{i},polys{i-1});
        plot(intersections{i-1},'FaceColor','green','FaceAlpha',1)
        
        intersect_grids{i-1} = gridPoly(intersections{i-1},5);
        for j = 1:length(intersect_grids{i-1})
            plot(intersect_grids{i-1}(j,1),intersect_grids{i-1}(j,2),'b.');
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
        plot(nodes{i}(node_count,1),nodes{i}(node_count,2),'r.')
        drawnow
        plot(nodes{i}(node_count,1),nodes{i}(node_count,2),'b.')
        drawnow

    end
    node_count = node_count + 1;
end

Pos = start;
for k = 1:size(intersections,2)
   Pos(k+1,:) = intersect_grids{k}(1,:); 
end
Pos(end+1,:) = goal;

T = 0:1:size(Pos,1)-1; % Time vector

[Ps] = MinimumSnapGenerator(T,Pos);

% use as path
path = Ps';

plot(path(:,1),path(:,2),'r')

end

