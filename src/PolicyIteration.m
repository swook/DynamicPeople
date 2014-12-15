function [ J_opt, u_opt_ind ] = PolicyIteration( P, G )
%POLICYITERATION Value iteration
%   Solve a stochastic shortest path problem by policy iteration.
%
%   [J_opt, u_opt_ind] = PolicyIteration(P, G) computes the optimal cost and
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
[MN,L] = size(G);
% u: a MN x 1 matrix that stores the policies for at each state
% initial policy must be feasible, otherwise J_k = Inf/NaN, for all k
u = 7 * ones(MN,1);
% find target cell
[target_ind,~] = find(G==0);
iter_states = 1:MN;
iter_states = iter_states( iter_states~=target_ind);
% policy iteration
while 1
    % evaluate
    % pij i,j in S\t must exclude termination state, otherwise singular
    % matrix, cannot solve J
    p = zeros(MN-1,MN-1);
    g = zeros(MN-1,1);
    % p A (MN-1) x (MN -1) matrix P(i,j) := probability from state i to
    % state j under an input control u
    for i = 1: numel(iter_states)
        p(i,:) = P(iter_states(i),iter_states,u(iter_states(i)));
        g(i) = G(iter_states(i),u(iter_states(i)));
    end
    % J(i)=g(i,u(i))+sum pij(u(i))J(j) i in S\t (exclude target cell)
    J_k = (eye(MN-1,MN-1) - p)\g;
    % transpose J_k to reuse ValueIteration code
    J_k = J_k';
    % must consider termination stage in maximization stage
    J_k(iter_states) = J_k;
    J_k(target_ind) = 0;
    % minimization stage
    J_candidates = zeros(L, MN);
    for l = 1:L
        % J_candidates(u, :): 1  x MN
        % P(:, :, u):         MN x MN
        % J_k:                1  x MN
        J_candidates(l, :) = J_k * P(:, :, l)';
    end
    J_candidates = J_candidates + G';
    [J_kp1, u] = min(J_candidates);
    % should break when J_kp1 == J_k, but due to numerical accuracy, this
    % equation can not be reached
    if (norm(J_kp1-J_k)<1e-2)
        break;
    end
end
J_opt = J_kp1;
u_opt_ind = u;
u_opt_ind(target_ind) = 0;

