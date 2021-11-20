function [inflated_occgrid] = inflateMap(occgrid, UAV_size, safety_factor)
%INFLATEMAP increases the obstacle size of an occupancy grid
%   occgrid is the old occupancy grid, UAV_size is the diameter (in
%   meters), and safety_factor is the amount to increase the size of the
%   UAV by (x/1.0)

inflated_occgrid = copy(occgrid);
inflate(inflated_occgrid, ceil(safety_factor*UAV_size))

end

