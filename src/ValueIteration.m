function [ J_opt, u_opt_ind ] = ValueIteration( P, G )
%VALUEITERATION Value iteration
%   Solve a stochastic shortest path problem by value iteration.
%
%   [J_opt, u_opt_ind] = ValueIteration(P, G) computes the optimal cost and
%   the optimal control input for each state of the state space.
%
%   Input arguments:
%
%       P:
%           A (MN x MN x L) matrix containing the transition probabilities
%           between all states in the state space for all attainable
%           control inputs. The entry P(i, j, l) represents the transition
%           probability from state i to state j if control input l is
%           applied.
%
%       G:
%           A (MN x L) matrix containing the stage costs of all states in
%           the state space for all attainable control inputs. The entry
%           G(i, l) represents the cost if we are in state i and apply
%           control input l.
%
%   Output arguments:
%
%       J_opt:
%       	A (1 x MN) matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (1 x MN) matrix containing the indices of the optimal control
%       	inputs for each element of the state space.

% put your code here
[MN, ~, L] = size(P);

k = 0;

% Initialisation
J0  = 0; % Guess based on estimated total cost to reach target?
J_k = J0 * ones(1, MN);

% Termination criterion
tolerance = 1e-2;

while 1
    k = k+1;
    % Do value calculation
    % min over u (control)
    % sum cost over all possible states
    %
    % TEMP
    % P:               MN x MN x L
    % G:               MN x L
    % J:               1 x MN
    % J_candidates(j): L x MN
    J_candidates = zeros(L, MN);
    for u = 1:L
        % J_candidates(u, :): 1  x MN
        % P(:, :, u):         MN x MN
        % J_k:                1  x MN
        J_candidates(u, :) = J_k * P(:, :, u)';
    end
    J_candidates = J_candidates + G';
    [J_kp1, I] = min(J_candidates);

    % Termination
    % J_k ~ J_kp1 (within tolerance)
    % TODO: u_opt_ind
    if norm(J_k - J_kp1, 2) <= tolerance
        break;
    end
    
    % Update for next iteration
    J_k = J_kp1;
end
J_opt     = J_k;
u_opt_ind = I;

end

