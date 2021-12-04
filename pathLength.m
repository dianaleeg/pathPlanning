function [len] = pathLength(points)
%PATHLENGTH Computes the length of a discrete path
%   points is a list of nx2 cartesian points

len = 0;
for i = 2:length(points)
    len = len + sqrt((points(i-1,1) - points(i,1))^2 + (points(i-1,2) - points(i,2))^2);
end

end

