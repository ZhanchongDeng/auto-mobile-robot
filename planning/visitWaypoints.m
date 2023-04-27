function [cmdV, cmdW, gotopt] = visitWaypoints(waypoints, pose, gotopt, closeEnough, epsilon)
    % VISITWAYPOINTS Visits a series of waypoints in order
    % Inputs:
    %   waypoints: waypoints: n x 2
    %   gotopt: index indicating which waypoint is currently going to
    %   closeEnough: finish threshold (m)
    %   pose: current pose of car: 1 x 3
    %   epsilon: turn radius
    % Outputs:
    %   cmdV: fwd velocity
    %   cmdW: angular velocity
    
    % check if the current waypoint is finished
    curr_waypoint = waypoints(gotopt, :);
    dist = norm(curr_waypoint - pose(1:2));
    if dist < closeEnough
        % arrive at current waypoint
        cmdV = 0;
        cmdW = 0;
        gotopt = gotopt + 1;
        return;
    end
    [cmdV, cmdW] = feedbackLin(curr_waypoint(1) - pose(1), curr_waypoint(2) - pose(2), pose(3), epsilon);

end
