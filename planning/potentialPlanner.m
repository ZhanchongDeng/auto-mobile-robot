function [dataStore] = potentialPlanner(Robot, max_time)
 %% Setup %%
    if nargin < 1
        disp('Error: TCP/IP port object not provided.')
    elseif nargin < 2
        max_time = 800;
    end
    
    try
        CreatePort = Robot.CreatePort;
    catch
        CreatePort = Robot;
    end
    
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
                   'GPS', []);
    SetFwdVelAngVelCreate(Robot, 0, 0)
    
    % Parameter Setup
    no_robot_count = 0;
    wheel2center = 0.16;
    max_v = 0.1;
    epsilon = 0.1;
%     map_file = 'hw6SphereApprox.txt';
%     map = importdata(map_file);
    map_file = 'labSphereMap_wall_orange.mat';
    map = load(map_file).map;
    goal = load(map_file).goal1;
    
    % No local min
    c_att = 0.5;
    c_rep = 20;
    Q = 0.1;
    
    % Local min:
%     c_att = 1e-5;
%     c_rep = 2;
%     Q = 2.3;
    
    finish_threshold = 0.1;
   
    tic
    while toc < max_time
        %% Store data %%
        [no_robot_count, dataStore] = readStoreSensorData(Robot, no_robot_count, dataStore);
                  
        %% Control %%
        if no_robot_count >= 3
            cmd_v = 0;
            cmd_w = 0;
        else
            pose = dataStore.truthPose(end, 2:end);
            if close_enough(pose(1,2), goal, finish_threshold)
                disp('Reach goal [' + goal(1) + ',' + goal(2) +']')
                break
            end
            [~, U_grad] = potentialPoint(map, goal, c_att, c_rep, Q, pose(1:2));
            cmd_x = -U_grad(1);
            cmd_y = -U_grad(2);
            [cmd_v, cmd_w] = feedbackLin(cmd_x, cmd_y, pose(3), epsilon);
        end
        [cmd_v, cmd_w] = limitCmds(cmd_v, cmd_w, max_v, wheel2center);
        SetFwdVelAngVelCreate(Robot, cmd_v, cmd_w);
    end
    SetFwdVelAngVelCreate(Robot, 0, 0);
end