function G = GjacDiffDrive(x, u)
    % GjacDiffDrive: output the jacobian of the dynamics. Returns the G matrix
    %
    %   INPUTS
    %       x            3-by-1 vector of pose
    %       u            2-by-1 vector [d, phi]'
    %
    %   OUTPUTS
    %       G            Jacobian matrix partial(g)/partial(x)
    %
    %   Cornell University
    %   Autonomous Mobile Robots
    %   Homework 4
    %   Last, First Name

    % dynamics is computed in the function files_HW2/integrateOdom.m
    theta = x(3);
    d = u(1);
    phi = u(2);

    if phi == 0
        G = [1 0 -1 * d * sin(theta);
            0 1 d * cos(theta);
            0 0 1];
    else
        r = d / phi;
        
        G = [1 0 r * (cos(theta + phi) - cos(theta));
            0 1 -1 * r * (sin(theta + phi) - sin(theta));
            0 0 1];
    end 
end