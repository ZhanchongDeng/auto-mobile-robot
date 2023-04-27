function [q_samp] = sample_pt_from_boundary(mapBoundary)
% SAMPLE_PT_FROM_BOUNDARY: uniformly sample a pt from boundary
% 
%   INPUTS:
%       mapBoundary     1 x 4 vector capturing the [x_bl y_bl x_tr y_tr] coordinates of the bottom left
%
%   OUTPUTS:
%       q_samp          1 x 2 sampled pt [x y]
%
    x = sample_from_uniform(1, [mapBoundary(1), mapBoundary(3)]);
    y = sample_from_uniform(1, [mapBoundary(2), mapBoundary(4)]);
    q_samp = [x y];
end
