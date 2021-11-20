function plotSolvedPath(occgrid, solnInfo, pthObj, plot_title, file_path)
%PLOTPATH plots the solved path over an occupancy grid
%   occgrid is the occupancy grid, title is the title of the plot, and the
%   file path is where the generated image is to be saved

figure
hold on
show(occgrid)
title(plot_title)
plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),'.-');
plot(pthObj.States(:,1),pthObj.States(:,2),'r-','LineWidth',2)
saveas(gcf,[pwd file_path])

end

