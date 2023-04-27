 function [U, U_grad] = potentialPoint(map, goal, c_att, c_rep, Q, point)
%   INPUTS:
%       map         map of the environment defined by circles. First row is the map boundary.
%                   k x 3 matrix [x_center y_center radius]
%       goal        goal point
%                   1 x 2 array [x_goal y_goal]
%       c_att       scaling factor for the atractive force (potential fun).
%       c_rep       scaling factor for the repulsive force (potential fun).
%       Q           influence range of the obstacles (real number)
%       point       a point to be evaluated [x, y],i.e., the center location of the robot

%   OUTPUTS:
%       U           Potential value at point
%       U_grad      1 x 2 array [gradient-x gradient-y] gradient of the potential function at point
%     max_potential = 1000;
    d = norm(point - goal);
    U_att = 1/2 * c_att * d^2;
    U_att_grad = c_att * (point - goal);
    U = U_att;
    U_grad = U_att_grad;
    for i=2:size(map, 1)
        d = norm(point - map(i, 1:2)) - map(i, 3);
        d_grad = (point - map(i, 1:2)) / norm(point - map(i, 1:2));
        d = max(d, 0.1*Q);
        if d <= Q
            U_rep = 1/2 * c_rep * (1/d - 1/Q)^2;
            U_rep_grad = c_rep * (1/Q - 1/d) * (1/d^2) * d_grad; 
        else
            U_rep = 0;
            U_rep_grad = 0;
        end
        U = U + U_rep;
        U_grad = U_grad + U_rep_grad;
    end
end
