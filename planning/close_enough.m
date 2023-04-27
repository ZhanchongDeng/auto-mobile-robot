function [is_close_enough] = close_enough(p1, p2, threshold)
% CLOSE_ENOUGH: given two points [x1, y1], [x2, y2], check whether their
% distance is within the threshold
is_close_enough = norm(p1 - p2) < threshold;