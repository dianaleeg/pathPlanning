function [grid] = gridPoly(poly, ppa)
%gridPoly Calculates a list of grid points within a polyshape
% poly = polyshape to calculate within
% ppa = number of points per unit area

n = sqrt(ppa);

[xv,yv] = boundary(poly);

inc_x = 1/n;
inc_y = 1/n;

interval_x = min(xv):inc_x:max(xv);
interval_y = min(yv):inc_y:max(yv);
[full_grid_x, full_grid_y] = meshgrid(interval_x,interval_y);

in_poly_points = inpolygon(full_grid_x,full_grid_y,xv,yv);
grid = [full_grid_x(in_poly_points),full_grid_y(in_poly_points)];

end

