function G = ComputeStageCostsI( stateSpace, controlSpace, disturbanceSpace, mazeSize, walls, targetCell )
%COMPUTESTAGECOSTSI Compute stage costs.
% 	Compute the stage costs for all states in the state space for all
%   attainable control inputs.
%
%   G = ComputeStageCostsI(stateSpace, controlSpace, disturbanceSpace,
%   mazeSize, walls, targetCell) computes the stage costs for all states in
%   the state space for all attainable control inputs.
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
%   Output arguments:
%
%       G:
%           A (MN x L) matrix containing the stage costs of all states in
%           the state space for all attainable control inputs. The entry
%           G(i, l) represents the cost if we are in state i and apply
%           control input l.

% Get dimensions
MN = size(stateSpace, 1);
L = size(controlSpace, 1);
width = mazeSize(1);
height = mazeSize(2);
numWalls = size(walls, 2)/2;

wallStarts = walls(:, mod(1:numWalls*2, 2) == 1); % Wall start positions
wallEnds = walls(:, mod(1:numWalls*2, 2) == 0);   % Wall end   positions
wallStarts = wallStarts';
wallEnds = wallEnds';

% Initialise stage cost matrix G
G = ones(MN, L);

% (k, l) is inf if l is infeasible at state k
% 1 penalty for each move (because we want to minimize moves)
% 0 at target cell (so that we want to move to and stay at target cell)
for k = 1:MN
    % Get current position (state)
    pos = stateSpace(k, :);

    % For each control input possible
    for l = 1:L
        % Get control input
        move = controlSpace(l, :);

        % If movement results in border or wall hit
        if (hitBorder(pos, move) || hitWall(pos, move))
            G(k, l) = Inf; % Infinite cost
        end
    end

    % If at target cell, no cost at all
    if isequal(pos, targetCell')
        G(k, 7) = 0;
        continue;
    end
end

% Check starting from one pos a move would lead to hitting a wall
function h = hitWall(pos, move)
    % move = [0, 0]
    if(move(1) == 0 && move(2) == 0)
        h = false;
        return
    end

    % K'*2 matrix storing relevant walls
    wallStartToCheck = [];
    wallEndToCheck = [];

    % Horizontal move
    if(move(2) == 0)
        switch move(1)
            case 1
                wallStartToCheck = [wallStartToCheck;pos + [0, -1]];
                wallEndToCheck = [wallEndToCheck;pos];
            case 2
                wallStartToCheck = [wallStartToCheck;pos + [0, -1]];
                wallEndToCheck = [wallEndToCheck;pos];
                wallStartToCheck = [wallStartToCheck;pos + [1, -1]];
                wallEndToCheck = [wallEndToCheck;pos + [1, 0]];
            case -1
                wallStartToCheck = [wallStartToCheck;pos + [-1, -1]];
                wallEndToCheck = [wallEndToCheck;pos + [-1, 0]];
            case -2
                wallStartToCheck = [wallStartToCheck;pos + [-1, -1]];
                wallEndToCheck = [wallEndToCheck;pos + [-1, 0]];
                wallStartToCheck = [wallStartToCheck;pos + [-2, -1]];
                wallEndToCheck = [wallEndToCheck;pos + [-2, 0]];
        end
    elseif(move(1)==0)
        % Vertical move
        switch move(2)
            case 1
                wallStartToCheck = [wallStartToCheck;pos + [-1, 0]];
                wallEndToCheck = [wallEndToCheck;pos];
            case 2
                wallStartToCheck = [wallStartToCheck;pos + [-1, 0]];
                wallEndToCheck = [wallEndToCheck;pos];
                wallStartToCheck = [wallStartToCheck;pos + [-1, 1]];
                wallEndToCheck = [wallEndToCheck;pos + [0, 1]];
            case -1
                wallStartToCheck = [wallStartToCheck;pos + [-1, -1]];
                wallEndToCheck = [wallEndToCheck;pos + [0, -1]];
            case -2
                wallStartToCheck = [wallStartToCheck;pos + [-1, -1]];
                wallEndToCheck = [wallEndToCheck;pos + [0, -1]];
                wallStartToCheck = [wallStartToCheck;pos + [-1, -2]];
                wallEndToCheck = [wallEndToCheck;pos + [0, -2]];
        end
    else
        % Diagonal move just need to check whether interested point is in
        % wall matrix
        pointToCheck = [];
        if (isequal(move, [1, 1]))
            pointToCheck = pos;
        elseif (isequal(move, [1, -1]))
            pointToCheck = pos + [0, -1];
        elseif (isequal(move, [-1, -1]))
            pointToCheck = pos + [-1, -1];
        elseif (isequal(move, [-1, 1]))
            pointToCheck = pos + [-1, 0];
        end
        h = (sum(ismember(walls', pointToCheck, 'rows')) > 0);
        return
    end

    % Check whether elements in wallToCheck is in walls
    % convert to K*2 matrix
    for i = 1:size(wallStartToCheck, 1)
        index = ismember(wallStarts, wallStartToCheck(i, :), 'rows');
        if sum(index) == 0
            continue;
        end
        found = ismember(wallEnds(index, :), wallEndToCheck(i, :), 'rows');
        if sum(found) > 0
            h = true;
            return;
        end
    end

    h = false;
end

% Check starting from one pos a move would lead to going out of border
function h = hitBorder(pos, move)
    % Calculate position at end of move
    pos = pos + move;

    % If outside of maze, have hit border
    if pos(1) < 1 || pos(1) > width || pos(2) < 1 || pos(2) > height
        h = true;
    else
        h = false;
    end
end

end

