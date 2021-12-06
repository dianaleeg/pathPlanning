close all
clear all
clc

start = [1, 1];
goal = [18, 6];

poly{1} = polyshape([1 2; 1 6; 6 5]);
poly{2} = polyshape([4 7; 4 2; 12 2; 10 7]);
poly{3} = polyshape([6 3; 11 4; 15 5; 14 3]);
poly{4} = polyshape([13 1; 13 7; 17 7; 17 1]);

[min_path, min_path_length] = SFC_trajGen(start, goal, poly);