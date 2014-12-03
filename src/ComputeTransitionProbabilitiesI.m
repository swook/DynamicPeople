function P = ComputeTransitionProbabilitiesI( stateSpace, controlSpace, disturbanceSpace, mazeSize, walls, targetCell )
%COMPUTETRANSITIONPROBABILITIESI Compute transition probabilities.
% 	Compute the transition probabilities between all states in the state
%   space for all attainable control inputs.
%
%   P = ComputeTransitionProbabilitiesI(stateSpace, controlSpace,
%   disturbanceSpace, mazeSize, walls, targetCell) computes the transition
%   probabilities between all states in the state space for all attainable
%   control inputs.
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
%       P:
%           A (MN x MN x L) matrix containing the transition probabilities
%           between all states in the state space for all attainable
%           control inputs. The entry P(i, j, l) represents the transition
%           probability from state i to state j if control input l is
%           applied.

% put your code here

numStates = size(stateSpace,1);
numInput = size(controlSpace,1);
width = mazeSize(1);
height = mazeSize(2);
numWalls = size(walls,2)/2;
wallStarts = walls(:,mod(1:numWalls*2,2)==1);
wallEnds = walls(:,mod(1:numWalls*2,2)==0);
wallStarts = wallStarts';
wallEnds = wallEnds';
P = zeros(numStates,numStates,numInput);
for k = 1:numStates
    pos = stateSpace(k,:);
    for l = 1:numInput
        control = controlSpace(l,:);
        pos_new = pos + control;
        % control input results in crossing the borders
        if hitBorder(pos,control)
            continue;
        end
        % control input results in hitting the wall
        if hitWall(pos,control)
            continue;
        end
        % input not hitting the wall, check whether hit wall after all possible
        % disturbance is applied
        for m = 1:size(disturbanceSpace,1)
            disturbance = disturbanceSpace(m,1:2);
            prob = disturbanceSpace(m,3);
            % if not hit wall after disturbance is applied
            if (~hitBorder(pos_new,disturbance) &&...
                    ~hitWall(pos_new,disturbance))
                pos_disturbed = pos_new + disturbance;
                nextState = (pos_disturbed(1)-1) * width + pos_disturbed(2);
                P(k,nextState,l) = P(k,nextState,l)+ prob;
            else
                % if hit wall after disturbance is applied, just increment
                % prob at the state after control input is applied
                nextState = (pos_new(1)-1) * width + pos_new(2);
                P(k,nextState,l) = P(k,nextState,l)+ prob;
            end
        end
    end

end
function h = hitWall(pos,move)
    % move = [0,0]
    if(move(1) == 0 && move(2) ==0)
        h = false;
        return
    end
    % K'*2 matrix storing relevant walls
    wallStartToCheck = [];
    wallEndToCheck = [];
    % horizontal move
    if(move(2) == 0)
        switch move(1)
            case 1
                wallStartToCheck = [wallStartToCheck;pos + [0,-1]];
                wallEndToCheck = [wallEndToCheck;pos];
            case 2
                wallStartToCheck = [wallStartToCheck;pos + [0,-1]];
                wallEndToCheck = [wallEndToCheck;pos];
                wallStartToCheck = [wallStartToCheck;pos + [1,-1]];
                wallEndToCheck = [wallEndToCheck;pos + [1,0]];
            case -1
                wallStartToCheck = [wallStartToCheck;pos + [-1,-1]];
                wallEndToCheck = [wallEndToCheck;pos + [-1,0]];
            case -2
                wallStartToCheck = [wallStartToCheck;pos + [-1,-1]];
                wallEndToCheck = [wallEndToCheck;pos + [-1,0]];
                wallStartToCheck = [wallStartToCheck;pos + [-2,-1]];
                wallEndToCheck = [wallEndToCheck;pos + [-2,0]];
        end
    elseif(move(1)==0)
        % vertical move
        switch move(2)
            case 1
                wallStartToCheck = [wallStartToCheck;pos + [-1,0]];
                wallEndToCheck = [wallEndToCheck;pos];
            case 2
                wallStartToCheck = [wallStartToCheck;pos + [-1,0]];
                wallEndToCheck = [wallEndToCheck;pos];
                wallStartToCheck = [wallStartToCheck;pos + [-1,1]];
                wallEndToCheck = [wallEndToCheck;pos + [1,1]];
            case -1
                wallStartToCheck = [wallStartToCheck;pos + [-1,-1]];
                wallEndToCheck = [wallEndToCheck;pos + [0,-1]];
            case -2
                wallStartToCheck = [wallStartToCheck;pos + [-1,-1]];
                wallEndToCheck = [wallEndToCheck;pos + [0,-1]];
                wallStartToCheck = [wallStartToCheck;pos + [-1,-2]];
                wallEndToCheck = [wallEndToCheck;pos + [0,-2]];
        end
    else
        % diagonal move just need to check whether interested point is in
        % wall matrix
        pointToCheck = [];
        if (move == [1,1])
            pointToCheck = pos;
        elseif (move == [1,-1])
            pointToCheck = pos + [0,-1];
        elseif (move == [-1,-1])
            pointToCheck = pos + [-1,-1];
        elseif (move == [-1,1])
            pointToCheck = pos + [-1,0];
        end
        h = ~ismember(walls',pointToCheck,'rows');
        return
    end
    % check whether elements in wallToCheck is in walls
    % convert to K*2 matrix
    for i = 1:size(wallStartToCheck,1)
        index = ismember(wallStarts,wallStartToCheck(i,:),'rows');
        if isempty(find(index,1))
            continue;
        end
        found = ismember(wallEnds(index,:),wallEndToCheck(i,:),'rows');
        if ~isempty(find(found,1))
            h = true;
            return;
        end
    end
    h = false;
end

function h = hitBorder(pos,move)
    pos = pos + move;
    if (pos(1)<1 ||pos(1)>width ||...
                pos(2)<1||pos(2)>height)
            h = true;
            return
    end
    h = false;
end
end
