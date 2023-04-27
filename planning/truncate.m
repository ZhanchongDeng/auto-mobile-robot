function [out] = truncate(in, lim)
% TRUNCATE: Given in and lim, return truncated in whose abs value is less
% than lim
if abs(in) > lim
    out = lim;
    if in < 0
        out = -1 * out;
    end
else
    out = in;
end