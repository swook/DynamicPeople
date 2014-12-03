function G = ComputeStageCostsII( stateSpace, controlSpace, disturbanceSpace, mazeSize, walls, targetCell, holes, resetCell, c_p, c_r )
%COMPUTESTAGECOSTSII Compute stage costs.
% 	Compute the stage costs for all states in the state space for all
%   attainable control inputs.
%
%   G = ComputeStageCostsII(stateSpace, controlSpace, disturbanceSpace,
%   mazeSize, walls, targetCell, holes, resetCell, wallPenalty, holePenalty)
%   computes the stage costs for all states in the state space for all
%   attainable control inputs.
%
%   Input arguments:
%
%       stateSpace:
%           A (MN x 2) matrix, where the i-th row represents the i-th
%           element of the state space. Note that the state space also
%           contains the target cell, in order to simplify state indexing.
%
%       controlSpace:
%           A (L x 2) matrix, where the l-th row represents the l-th
%           element of the control space.
%
%       disturbanceSpace:
%           A (S x 3) matrix 'disturbanceSpace', where the first two
%           columns of each row represent an element of the disturbance
%           space, and the third column represents the corresponding
%           probability.
%
%       mazeSize:
%           A (1 x 2) matrix containing the width and the height of the
%           maze in number of cells.
%
%   	walls:
%          	A (2 x 2K) matrix containing the K wall segments, where the start
%        	and end point of the k-th segment are stored in column 2k-1
%         	and 2k, respectively.
%
%    	targetCell:
%          	A (2 x 1) matrix describing the position of the target cell in
%         	the maze.
%
%    	holes:
%         	A (2 x H) matrix containg the H holes of the maze. Each column
%         	represents the position of a hole.
%
%   	resetCell:
%         	A (2 x 1) matrix describing the position of the reset cell in
%           the maze.
%
%       c_p:
%         	Penalty (in time steps) that we get every time the ball bounces
%           into a wall.
%
%       c_r:
%           Penalty (in time steps) that we get every time the ball falls
%           into a hole.
%
%   Output arguments:
%
%       G:
%           A (MN x L) matrix containing the stage costs of all states in
%           the state space for all attainable control inputs. The entry
%           G(i, l) represents the cost if we are in state i and apply
%           control input l.

% put your code here

end

