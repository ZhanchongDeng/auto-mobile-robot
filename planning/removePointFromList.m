function [removed_ls] = removePointFromList(ls, pt)
% REMOVEPOINTFROMLIST: remove pt from ls, return the new ls
%
%   INPUTS:
%       ls:         k-by-2
%       pt:         1-by-2
%
%   OUTPUTS:
%       removed_ls: (k-1)-by-2
    idx = ismember(ls, pt, 'row');
    removed_ls = ls(~idx, :);
end