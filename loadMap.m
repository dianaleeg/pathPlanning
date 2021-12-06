function [occgrid, occgrid_unscaled] = loadMap(filename, map_size)
%LOAD_MAP imports an image with the specified filename (text array) and
%map_size (single value for square image) and converts it to
%an occupancy grid map. It returns this map.

image = imread(filename); % read in scan data
grayimage = rgb2gray(image); % convert  to grey scale
bwimage = grayimage < 100; % convert to black and white

bwimage_sclaed = imresize(bwimage,[map_size map_size]);

size_exp = map_size(1); %mm
size_act = size(bwimage,1);
scale = (size_act / size_exp);

occgrid = binaryOccupancyMap(bwimage_sclaed); % load in to occupancy grid
occgrid_unscaled = binaryOccupancyMap(bwimage, scale); % load in to occupancy gridz
end