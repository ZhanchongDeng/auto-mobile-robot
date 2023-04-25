function H = HjacDepth(x, map, sensor_pos, K)
    % HjacDepth: output the jacobian of the depth measurement. Returns the H matrix
    %
    %   INPUTS
    %       x            3-by-1 vector of pose
    %       map          of environment, n x [x1 y1 x2 y2] walls
    %       sensor_pos   sensor position in the body frame [1x2]
    %       K            number of measurements (rays) the sensor gets between 27 to -27 
    %
    %   OUTPUTS
    %       H            Kx3 jacobian matrix
    %
    %   Cornell University
    %   Autonomous Mobile Robots
    %   Homework 4
    %   Last, First Name
    
    % forward finite difference to get jacobian
    delta = 0.0001;
    H = zeros(K,3);
    angles = linspace(27 / 180 * pi, -27 / 180 * pi, K).';
    z = depthPredict(x, map, sensor_pos, angles);

    for i = 1:3
        x_prime = x;
        x_prime(i) = x_prime(i) + delta;
        z_prime = depthPredict(x_prime, map, sensor_pos, angles);
        H(:,i) = (z_prime - z) / delta;
    end
end