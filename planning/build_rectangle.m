function [rec] = build_rectangle(x1, y1, x2, y2, r)
% BUILD_RECTANGLE: Return a rectangle polygon surrounding the line 
% 
%   INPUTS:
%       [x1 y1 x2 y2]   line
%       r               robot radius
%
%   OUTPUTS:
%       rec             5-by-2 vertices of rectangle
    %% Compute the theta, and the unit vector orthogonal to the line
    theta = atan2(y2-y1, x2-x1);
    hori = [cos(theta), sin(theta)];
    vertical = [cos(theta + pi/2), sin(theta + pi/2)];
    
    %% Compute the four vertices
    p1 = [x1 y1] + r * vertical - r * hori;
    p2 = [x1 y1] - r * vertical - r * hori; 
    p3 = [x2 y2] - r * vertical + r * hori; 
    p4 = [x2 y2] + r * vertical + r * hori; 
    rec = vertcat(p1,p2,p3,p4,p1);
    
end
