function P = ComputeTransitionProbabilitiesII( stateSpace, controlSpace, disturbanceSpace, mazeSize, walls, targetCell, holes, resetCell )
%COMPUTETRANSITIONPROBABILITIESII Compute transition probabilities.
% 	Compute the transition probabilities between all states in the state
%   space for all attainable control inputs.
%
%   P = ComputeTransitionProbabilitiesII(stateSpace, controlSpace,
%   disturbanceSpace, mazeSize, walls, targetCell, holes, resetCell)
%   computes the transition probabilities between all states in the state
%   space for all attainable control inputs.
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
%   Output arguments:
%
%       P:
%           A (MN x MN x L) matrix containing the transition probabilities
%           between all states in the state space for all attainable
%           control inputs. The entry P(i, j, l) represents the transition
%           probability from state i to state j if control input l is
%           applied.

% Difference to task 1:
% 1. Penalty cp if the ball hit a wall after control input (due to disturbance)
% 2. Penalty in falling into a hole. Check for pos i and control input u,
%    whether the ball would fall into a hole.
%    If yes, set pij(u) = 1.00 i:= hole j:=reset

% judge whether a move from current position will lead to falling into a
% hole
% Check starting from one pos a move would lead to hitting a wall

% Get dimensions
numStates = size(stateSpace, 1);
numInput = size(controlSpace, 1);
width = mazeSize(1);
height = mazeSize(2);
numWalls = size(walls, 2)/2;

wallStarts = walls(:, mod(1:numWalls*2, 2) == 1); % Wall start positions
wallEnds = walls(:, mod(1:numWalls*2, 2) == 0);   % Wall end   positions
wallStarts = wallStarts';
wallEnds = wallEnds';

% Get state index for reset cell
resetState = (resetCell(1)-1)*height + resetCell(2);

% Initialise transition probabilities matrix P
P = zeros(numStates, numStates, numInput);

for k = 1:numStates
    % Get current position (state)
    pos = stateSpace(k, :);

    % If at target cell, stay at target cell
    if isequal(pos, targetCell')
        P(k, k, :) = zeros(numInput, 1);
        P(k, k, 7) = 1;
        continue;
    end

    % Ball will never move from hole to another cell
    if fallInHoles(pos, [0, 0])
        continue;
    end

    % For each control input possible
    for l = 1:numInput
        % Get control input
        control = controlSpace(l, :);

        % Calculate new position
        pos_new = pos + control;

        % Control input results in crossing the borders or hitting a wall
        if hitBorder(pos, control) || hitWall(pos, control)
            continue;
        end

        % Control input results in falling into a hole
        if fallInHoles(pos, control)
            P(k, resetState, l) = 1.0;
            continue;
        end

        % Input not hitting the wall, check whether hit wall after all possible
        % disturbance is applied
        for m = 1:size(disturbanceSpace, 1)
            disturbance = disturbanceSpace(m, 1:2);
            prob = disturbanceSpace(m, 3);

            % if hit wall or border after disturbance is applied, just increment
                % prob at the state after control input is applied
            if hitBorder(pos_new, disturbance) || hitWall(pos_new, disturbance)
                nextState = (pos_new(1)-1) * height + pos_new(2);
                P(k, nextState, l) = P(k, nextState, l)+ prob;
            else
                % If not hit a wall or border after disturbance is applied
                pos_disturbed = pos_new + disturbance;

                % If fall into a hole
                if fallInHoles(pos_new, disturbance)
                    P(k, resetState, l) = P(k, resetState, l) + prob;
                else
                    nextState = (pos_disturbed(1)-1) * height + pos_disturbed(2);
                    P(k, nextState, l) = P(k, nextState, l)+ prob;
                end

            end
        end
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

