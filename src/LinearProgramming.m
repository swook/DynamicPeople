function [ J_opt, u_opt_ind ] = LinearProgramming( P, G )
%LINEARPROGRAMMING Value iteration
%   Solve a stochastic shortest path problem by linear programming.
%
%   [J_opt, u_opt_ind] = LinearProgramming(P, G) computes the optimal cost
%   and the optimal control input for each state of the state space.
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

% Get dimensions
[MN,L] = size(G);

% f must be -ones since we want max sum(J) and linprog solution is
% min f'*x
f = -ones(MN - 1, 1);

% Find target cell
[target_ind,~] = find(G==0);

% Get all non-target states
iter_states = 1:MN;
iter_states = iter_states( iter_states~=target_ind);

% (MN-1) x L constraints
% p: a ((MN-1) x L) x (MN-1) matrix. Pij(u) is stored in p((u-1)*(MN-1) + i,j)
% g: a ((MN-1) x L) x 1 vector. g(i,u) is stored in g((u-1)*(MN-1) +i)
A = [];
g = [];

% Construct A matrix and g vector for use in constrained inequality:
%  A * x <= g
for u = 1:L                       % For each control input
    for i = 1: numel(iter_states) % For each non-target state

        % Ignore infeasible input control
        if G(iter_states(i),u) == Inf
            continue;
        end

        % Append entries to A and g
        p = P(iter_states(i),iter_states,u);
        g(end+1) = G(iter_states(i),u);
        curr_row = zeros(1,MN-1);
        curr_row(i) = 1;
        A(end+1,:) = curr_row - p;
    end
end

% Calculate optimal J
J_opt = linprog(f,A,g);
J_opt = J_opt';

% must consider termination stage in maximization stage
J_opt(iter_states) = J_opt;
J_opt(target_ind) = 0;

% Calculate optimal control (one step in value iteration)
J_candidates = zeros(L, MN);
for l = 1:L
    % J_candidates(u, :): 1  x MN
    % P(:, :, u):         MN x MN
    % J_k:                1  x MN
    J_candidates(l, :) = J_opt * P(:, :, l)';
end
J_candidates = J_candidates + G';

% Find index for optimal control
[~, u_opt_ind] = min(J_candidates);

end

