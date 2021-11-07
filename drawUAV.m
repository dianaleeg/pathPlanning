function drawUAV(state, figure_handle)
%DRAW_UAV Draws a 2-d representation of a UAV given its position and
%orientation in space. State = [x,y,th]

set(0, 'CurrentFigure', figure_handle)
hold on

c_x = state(1);
c_y = state(2);
th = state(3);

center = [c_x, c_y];

arm_length = 0.1;
prop_dia = 0.05;
plot(c_x,c_y,'k.','MarkerSize',20)
end_1 = drawVectorFromAngle2D(center, arm_length, th+pi/2, 'b');
end_2 = drawVectorFromAngle2D(center, arm_length, th+pi, 'r');
end_3 = drawVectorFromAngle2D(center, arm_length, th+((3*pi)/2), 'g');
end_4 = drawVectorFromAngle2D(center, arm_length, th, 'y');

viscircles([end_1; end_2; end_3; end_4], ones(4,1)*prop_dia/2, 'Color', 'k');

end

