function endP_global = drawVectorFromAngle2D(startP, length, angle, style)
    endP(1) = length*cos(angle); % calculate local end position of vector
    endP(2) = length*sin(angle);
    
    quiver(startP(1),startP(2),endP(1),endP(2),style,'ShowArrowHead','off','AutoScale','off','LineWidth',3); % draw vector
    
    endP_global(1) = startP(1) + endP(1); % calculate global end position of vector
    endP_global(2) = startP(2) + endP(2);
end
