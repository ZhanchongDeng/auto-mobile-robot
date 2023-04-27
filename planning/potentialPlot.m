function [h_potential_field, U, U_grad] = potentialPlot(map, goal, c_att, c_rep, Q, pointsX, pointsY)
%   INPUTS:
%       map         map of the environment defined by circles. First row is the map boundary.
%                   k x 3 matrix [x_center y_center radius]
%       goal        goal point
%                   1 x 2 array [x_goal y_goal]
%       c_att       scaling factor for the atractive force (potential fun).
%       c_rep       scaling factor for the repulsive force (potential fun).
%       Q           influence range of the obstacles (real number)
%       pointsX     x of a list of points to be evaluated 
%       pointsY     y of a list of points to be evaluated
%
%   OUTPUTS:
%       hplot       plot handler
    %% Set up and calculate U and U_grad
    grad_lim = 10000;
    n = size(pointsX, 2);
    m = size(pointsX, 1);
    num_pt = size(pointsX(:), 1);
    points = zeros(num_pt, 2);
    points(:, 1) = pointsX(:);
    points(:, 2) = pointsY(:);
    U = zeros(num_pt, 1);
    U_grad= zeros(size(points));
    for i=1:num_pt
        [U(i), U_grad(i, :)] = potentialPoint(map, goal, c_att, c_rep, Q, points(i, :));
        U_grad(i, 1) = truncate(U_grad(i, 1), grad_lim);
        U_grad(i, 2) = truncate(U_grad(i, 2), grad_lim);
    end
    %% Plot potential field based on U
%     pointsX = reshape(points(:, 1), [], 2);
%     pointsY = reshape(points(:, 2), [], 2);
    U_reshape = reshape(U, m, n);
    h_potential_field = figure();
    mesh(pointsX, pointsY, U_reshape);
    title('Potential Field, Catt = ' + string(c_att) + ', Crep = ' + string(c_rep) + ', Q = ' + string(Q))
    xlabel('x (m)')
    ylabel('y (m)')
    zlabel('potential')
    
    %% Plot vector field based on U_grad
    h_vector_field = figure();
    U_grad_X_reshape = reshape(U_grad(:, 1), m, n);
    U_grad_Y_reshape = reshape(U_grad(:, 2), m, n);
  
    pboundary = circle(map(1, 1:2), map(1, 3), 1000);
    hold on
    for i=2:size(map, 1)
        pobstacle = circle(map(i, 1:2), map(i, 3), 1000, 'g-');
        hold on
    end
  
    pvector = quiver(pointsX, pointsY, U_grad_X_reshape, U_grad_Y_reshape);
    xlabel('x (m)')
    ylabel('y (m)')
    legend([pboundary, pobstacle, pvector], ...
            'Boundary', 'Obstacles', 'Gradient')
    title('Vector Field, Catt = ' + string(c_att) + ', Crep = ' + string(c_rep) + ', Q = ' + string(Q))
    hold off
end