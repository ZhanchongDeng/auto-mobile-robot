function [q_samp] = sample_from_Qfree(obs, mapBoundary, goal)
% SAMPLE_FROM_QFREE: sample a pt from Q_free
%
%   INPUTS:
%       obs             1-by-n cell array of n obstacles, each obstacle is k-by-2
%       mapBoundary     1 x 4 vector capturing the [x_bl y_bl x_tr y_tr] coordinates of the bottom left 
%                       and top right corner of the workspace respectively
%       goal            1 x 2 array [x y], for goal point
%
%   OUTPUTS:
%       q_samp          1 x 2 array [x y] sample point
%
    %% Decide which sampling mode to use
    mode = randi([1, 3]);

    %% Sampling
    if mode == 1
        % Uniform sampling
        q_samp = sample_pt_from_boundary(mapBoundary);
        while ~check_pt_free(q_samp, obs)
            q_samp = sample_pt_from_boundary(mapBoundary);
        end

    elseif mode == 2
        % Set q_samp to be goal
        q_samp = goal;
    else
        % Sample from N(goal, sigma)
        sigma = 10;
        cov = eye(2) * sigma;
        q_samp = mvnrnd(goal, cov);
        while ~check_pt_free(q_samp, obs)
            q_samp = mvnrnd(goal, cov);
        end
    end
end