function TestSphereWorldPot(map, goal, c_att, c_rep, Q, points)
% TESTSPHEREWORLD
% Test function for MAE 4180/5180, CS 4758/5758, ECE 4180/5772 Homework 6. 
% Plots the potential field.
%
%       INPUTS:
%           map         map of the environment defined by circles.
%                       k x 3 matrix [x_center y_center radius]
%           goal        goal point
%                       1 x 2 array [x_goal y_goal]
%           c_att       scaling factor for the attractive force (potential fun).
%           c_rep       scaling factor for the repulsive force (potential fun).
%           Q           influence range of the obstacles (real number)
%           points      list of points to be evaluate
%                       n x 2 matrix [x, y]
%
%                       
%       OUTPUTS:
%           none
%       Figures Created:
%           Figure 1    Potential field 
%
% Autonomous Mobile Robots
% 

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%       POTENTIAL FUNCTION
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

% STUDENTS: Call the function potentialPlot to generate the potential field
% plot in the specified points
xlim = [min(points(:,1)), max(points(:,1))];
ylim = [min(points(:,2)), max(points(:,2))];
n = 100;
m = 100;
[pointsX, pointsY] = meshgrid(linspace(xlim(1), xlim(2), n), linspace(ylim(1), ylim(2), m));
[h_potential_field, U, U_grad] = potentialPlot(map, goal, c_att, c_rep, Q, pointsX, pointsY);

%END
end

