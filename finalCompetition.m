function [dataStore] = finalCompetition(Robot, maxTime, offset_x, offset_y)
% FINALCOMPETITION: main control function for the final competition
    %% ==== Robot setup ==== %%
    if nargin < 1
        disp('Error: TCP/IP port object not provided.')
    elseif nargin < 2
        maxTime = 800;
    end
    
    try
        CreatePort = Robot.CreatePort;
    catch
        CreatePort = Robot;
    end
    SetFwdVelAngVelCreate(Robot, 0, 0)
    
    %% ==== Data setup ==== %%
    global dataStore;
    
    dataStore = struct('truthPose', [], ...
                   'odometry', [], ...
                   'rsdepth', [], ...
                   'bump', [], ...
                   'beacon', [], ...
                   'deadReck', [], ...
                   'ekfMu', [], ...
                   'ekfSigma', [], ...
                   'particles', [], ...
                   'particle_weights', [], ...
                   'particles_bar', [], ...
                   'GPS', [], ...
                   'visitedWaypoints', []);
    
    % Parameter Setup
    noRobotCount = 0;
    wheel2Center = 0.16;
    radius = 0.15;
    maxV = 0.1;
    epsilon = 0.2;
    gotopt = 1;
    closeEnough = 0.2;
    
    % Load map
    map_file = 'practiceMap_4credits.mat';
    load(map_file, 'beaconLoc', 'ECwaypoints', 'map', 'optWalls', 'waypoints');
    
    %% Plot map
    pmap = plotmapopts(map, 'color', 'green', 'LineWidth', 1);

end