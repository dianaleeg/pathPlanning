close all
clear all
clc

start = [1, 1];
goal = [18, 6];

poly_list{1} = [1 2; 1 6; 6 5];
poly_list{2} = [4 7; 4 2; 12 2; 10 7];
poly_list{3} = [6 3; 11 4; 15 5; 14 3];
poly_list{4} = [13 1; 13 7; 17 7; 17 1];

[min_path, min_path_length] = SFC_trajGen(start, goal, poly_list);