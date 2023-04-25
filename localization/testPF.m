function [particlesDepth, weightsDepth] = ...
    testPF(particles_prev, weights_prev, u_prev, z_depth, map, sensor_pos, n_rs_rays)
% testEKF: Performs one step of the extended Kalman Filter, outputs the belief given previous belief
%
%   INPUTS
%       mu_prev         previous vector of pose state (mu(t-1))
%       weights_prev    previous weights
%       u_prev          previous command [d; phi]    
%       sigma_prev      previous covariance matrix
%       z_gps           current gps measurement vector
%       z_depth         depth measurement vector
%       map             map of the environment
%       sensor_pos      sensor position in the robot frame [x y]
%       n_rs_rays       number of evenly distributed realsense depth rays
%                       (27...-27) degrees
%
%   OUTPUTS
%       particlesDepth    particles after PF update with depth
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 4
%   Last, First Name
    
    angles_degree = linspace(27, -27, n_rs_rays);
    angles = angles_degree * pi / 180;
    % create function pointers
    dynamics = @(x,u) integrateOdom(x, u(1), u(2));
    sensorDepth = @(x) depthPredict(x, map, sensor_pos, angles.');

    % estimate the state with depth data using EKF
    [particlesDepth, weightsDepth] = PF(particles_prev, weights_prev, u_prev, z_depth, dynamics, sensorDepth);
end