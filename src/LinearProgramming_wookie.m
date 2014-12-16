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

[MN, ~, L] = size(P);
MN_L = MN*L;

% Contraints must be valid for all i and u
% This means all available values of i and u must be encoded into Ax = b
%
% There are MN   possible states (i)
% There are L    possible control inputs per state
% There are MNxL possible control input combinations
%
% Inequality A*x <= b must hold for solution of min f'*x
% Therefore f must be -ones since we want max sum(J)
% where x: J'
%       b: G
%       A: I - P
%
% f = -1     (MN   x L)
% A = I - P  (MNxL x MN)
% x = J'     (MN   x 1)
% b = G      (MNxL x 1)

f = -ones(MN, 1);

P_ = reshape(P, MN_L, MN);
I  = eye(MN_L, MN);
A  = I - P_;

b = reshape(G, MN_L, 1);

% Set impossible controls to 0
bad_ind = find(b == Inf);
A(bad_ind, :) = [];
b(bad_ind) = [];

% Run linear programme solver
x = linprog(f, A, b);
J_opt = x'

% Solve next value iteration step to find optimal control inputs
J_candidates = zeros(L, MN);
for u = 1:L
    % J_candidates(u, :): 1  x MN
    % P(:, :, u):         MN x MN
    % J_opt:              1  x MN
    J_candidates(u, :) = J_opt * P(:, :, u)';
end
J_candidates = J_candidates + G';
[~, u_opt_ind] = min(J_candidates);

end

