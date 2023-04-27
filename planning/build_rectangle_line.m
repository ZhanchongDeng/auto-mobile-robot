function [line1, line2] = build_rectangle_line(x1, y1, x2, y2, r)
% BUILD_RECTANGLE: Return a rectangle polygon surrounding the line 
% 
%   INPUTS:
%       [x1 y1 x2 y2]   line
%       r               robot radius
%
%   OUTPUTS:
%       rec             5-by-2 vertices of rectangle
    %% Compute the theta, and the unit vector orthogonal to the line
    theta = atan2(y2-y1, x2-x1) + pi/2;
    v = [cos(theta), sin(theta)];
    
    %% Compute the four vertices
    p1 = [x1 y1] + r * v;
    p2 = [x1 y1] - r * v; 
    p3 = [x2 y2] - r * v; 
    p4 = [x2 y2] + r * v; 
    line1 = horzcat(p1, p4);
    line2 = horzcat(p2, p3);
    
end
