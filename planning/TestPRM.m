function TestPRM(map,mapBoundary,n_PRM,q_start,q_goal)
% Test function for MAE 4180/5180, CS 4758/5758, ECE 4180/5772 HW 6 Extra Credit
%
%       INPUTS:
%           map         	  file name for text file representing the obstacles in the workspace
%                             for example workspace = 'hw6b.txt'. Each row in this file contains the vertices
%                             of one polygonal obstacle: v1x, v1y, v2x, v2y, etc. The vertices are given in
%                             counterclockwise order. If an obstacle has fewer vertices, unused entries 
%                             in the line will contain the value zero
%           mapBoundary       1 x 4 vector capturing the [x_bl y_bl x_tr y_tr] coordinates of the bottom left 
%                             and top right corner of the workspace respectively  
%           n_PRM             number of vertices in the PRM     
%           q_start 		  1x2 array [x y],vector of start point 
%           q_goal            1x2 array [x y],vector of goal point 
%       OUTPUTS:
%           plot of the PRM and path between start to goal

end