function [q_new] = extend(q_samp, q_near, step_size, obs)
% EXTEND: find a new point on the line of q_near, q_samp that is roughly
% step_size away from q_near, in obs
%
%   INPUTS:
%       q_samp      1-by-2 sample pt
%       q_near      1-by-2 pt on RRT nearest to q_samp
%       step_size   float indicating the size of the step
%        obs        1-by-n cell array of n obstacles, each obstacle is k-by-2
%
%   OUTPUTS:
%       q_new       1-by-2 new point to be connected to RRT
%
    line_length = norm(q_samp - q_near);
    unit_direction = (q_samp - q_near) / line_length;
    q_new = q_near + step_size * unit_direction;

%     %% Make sure q_new is in Qfree
%     while ~check_pt_free(q_new, obs)
%         step_size = step_size / 2;
%         q_new = q_near + step_size * unit_direction;
%     end
% 
% %% Keep adding step_size to q_new until it 
% prev_q_new = q_new;
% while check_pt_free(q_new, obs) && norm(q_new - q_near) < line_length
%     prev_q_new = q_new;
%     q_new = q_new + step_size * unit_direction;
% end
% q_new = prev_q_new;
end