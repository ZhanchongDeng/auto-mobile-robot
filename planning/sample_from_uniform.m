function [row_matrix] = sample_from_uniform(M, limits)
    % SAMPE_FROM_UNIFORM: samples M numbers from a uniform distribution defined by limits = [lower_bound, upper_bound]     
    row_matrix = limits(1) + (limits(2) - limits(1)) .* rand(1, M);
end