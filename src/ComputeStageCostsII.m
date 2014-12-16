function G = ComputeStageCostsII( stateSpace,  controlSpace,  disturbanceSpace,  mazeSize,  walls,  targetCell,  holes,  resetCell,  c_p,  c_r )
%COMPUTESTAGECOSTSII Compute stage costs.
% 	Compute the stage costs for all states in the state space for all
%   attainable control inputs.
%
%   G = ComputeStageCostsII(stateSpace,  controlSpace,  disturbanceSpace,
%   mazeSize,  walls,  targetCell,  holes,  resetCell,  wallPenalty,  holePenalty)
%   computes the stage costs for all states in the state space for all
%   attainable control inputs.
%
%   Input arguments:
%
%       stateSpace:
%           A (MN x 2) matrix,  where the i-th row represents the i-th
%           element of the state space. Note that the state space also
%           contains the target cell,  in order to simplify state indexing.
%
%       controlSpace:
%           A (L x 2) matrix,  where the l-th row represents the l-th
%           element of the control space.
%
%       disturbanceSpace:
%           A (S x 3) matrix 'disturbanceSpace',  where the first two
%           columns of each row represent an element of the disturbance
%           space,  and the third column represents the corresponding
%           probability.
%
%       mazeSize:
%           A (1 x 2) matrix containing the width and the height of the
%           maze in number of cells.
%
%   	walls:
%          	A (2 x 2K) matrix containing the K wall segments,  where the start
%        	and end point of the k-th segment are stored in column 2k-1
%         	and 2k,  respectively.
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
%           G(i,  l) represents the cost if we are in state i and apply
%           control input l.

% Get dimensions
MN = size(stateSpace,  1);
L = size(controlSpace, 1);
width = mazeSize(1);
height = mazeSize(2);
numWalls = size(walls, 2)/2;
D = size(disturbanceSpace, 1);

wallStarts = walls(:,  mod(1:numWalls*2,  2) == 1); % Wall start positions
wallEnds = walls(:,  mod(1:numWalls*2,  2) == 0);   % Wall end   positions
wallStarts = wallStarts';
wallEnds = wallEnds';

% G(k, l) = Inf if l is infeasible at state k (hits wall, border)
% G(k, l) = E{g(k, l, w)}
% G(k, l) = 1 + c_r if ball fall into a hole
% G(k, l) = 1 + c_r if ball fall into a hole after control input
%
% 1 penalty for each move (because we want to minimize moves)
% 0 at target cell (so that we want to move to and stay at target cell)
G = ones(MN, L);
for k= 1:MN
    % Get current position (state)
    pos = stateSpace(k, :);

    % For each control input possible
    for l = 1:L
        % Get control input
        move = controlSpace(l, :);

        % G(k, l) = Inf if u leads directly to hitting a wall (infeasible
        % move)
        if (hitBorder(pos, move) || hitWall(pos, move))
           G(k, l) = Inf;
        elseif fallInHoles(pos, move)
            % G(k, l) = 1 + c_r if ball fall into a hole
            G(k, l) = G(k, l) + c_r;
        else
            % If disturbance results in bouncing off wall or falling into hole
            % G(k, l) = E_w{g(k, l, w)}
            kost = 0;
            pos_new = pos + move;
            for d = 1:D
                disturbance = disturbanceSpace(d, 1:2);
                if hitBorder(pos_new, disturbance) || hitWall(pos_new, disturbance)
                    % Bounces off of wall
                    kost = kost + c_p * 0.2;
                elseif fallInHoles(pos_new, disturbance)
                    % Fall into hole
                    kost = kost + c_r * 0.2;
                end
            end
            G(k, l) = kost + 1;
        end
    end

    % Set target zero cost
    if isequal(pos, targetCell')
        G(k, 7) = 0;
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

% Check whether a move leads to falling into a hole
function h = fallInHoles(pos, move)
    if isequal(move, [2, 0])
        posToCheck = [pos + [1, 0];pos + [2, 0]];
    elseif isequal(move, [-2, 0])
        posToCheck = [pos + [-1, 0];pos + [-2, 0]];
    elseif isequal(move, [0, 2])
        posToCheck = [pos + [0, 1];pos + [0, 2]];
    elseif isequal(move, [0, -2])
        posToCheck = [pos + [0, -1];pos + [0, -2]];
    else
        posToCheck = pos + move;
    end

    % check whether elements in posToCheck is in holes
    for i = 1:size(posToCheck, 1)
        index = ismember(holes', posToCheck(i, :), 'rows');
        if sum(index) > 0
            h = true;
            return;
        end
    end
    h = false;
end

end

