% ScriptI.m
%
% Matlab script that calls all the functions for computing the optimal cost
% and policy of the given problem.
%
% Dynamic Programming and Optimal Control
% Fall 2014
% Programming Exercise
%
% --
% ETH Zurich
% Institute for Dynamic Systems and Control
% Robin Ritz
% rritz@ethz.ch
%
% --
% Revision history
% [18.11.2014, RR]    first version
% [19.11.2014, DB]    named plots (value iteration, ...)
%

%% clear workspace and command window
s=dbstatus;
save('myBreakpoints.mat', 's');
clear all
load('myBreakpoints.mat');
dbstop(s);
close all;
clc;

%% define problem size and generate maze
shouldGenerateMaze = false;
if shouldGenerateMaze
	mazeSize = [ 2, 4 ];
	[ walls, targetCell, ~, ~ ] = GenerateMaze( mazeSize( 1 ), ...
        mazeSize( 2 ), false );
    % This generates a new random maze.
else
    load( 'pregeneratedMazeI.mat' );
    % In order to save time we can just load a pre-generated maze.
end
PlotMaze( 1, mazeSize, walls, targetCell, [], [] );

%% load control and disturbance space
load( 'controlSpace.mat' );
% This loads a (L x 2) matrix 'controlSpace', where the l-th row represents
% the l-th element of the control space. Note that the control space U(i)
% for a particular state might be reduced due to the walls of the maze.
load( 'disturbanceSpace.mat' );
% This loads a (S x 3) matrix 'disturbanceSpace', where the first two
% columns of each row represent an element of the disturbance space, and
% the third column represents the corresponding probability. Note that the
% probabilities might have to be adjusted if a wall blocks the movement of
% the ball.

%% generate state space
stateSpace = [];
for i = 1 : mazeSize( 1 )
    for j = 1 : mazeSize( 2 )
        index = ( i - 1 ) * mazeSize( 2 ) + j;
        stateSpace( index, : ) = [ i, j ];
    end
end
% This generates a (MN x 2) matrix 'stateSpace', where each row represents
% an element of the state space. Note that the state space also contains
% the target cell, in order to simplify state indexing.

%% compute transition probabilities
P = ComputeTransitionProbabilitiesI( stateSpace, controlSpace, ...
    disturbanceSpace, mazeSize, walls, targetCell );
% This computes the transition probabilities between all states in the
% state space for all attainable control inputs.
% The transition probability matrix has the dimension (MN x MN x L), i.e.
% the entry P(i, j, l) representes the transition probability from state i
% to state j if control input l is applied.
% If a control input l is not feasible for a particular state i, the
% transition  probabilities to all other states j can be set to zero.

%% compute stage costs
G = ComputeStageCostsI( stateSpace, controlSpace, disturbanceSpace, ...
    mazeSize, walls, targetCell );
% This computes the stage costs for all states in the state space for all
% attainable control inputs.
% The stage cost matrix has the dimension (MN x L), i.e. the entry G(i, l)
% represents the cost if we are in state i and apply control input l.
% If a control input l is not feasible for a particular state i, the stage
% cost can be set to infinity.

%% solve stochastic shortest path problem
[ J_opt_vi, u_opt_ind_vi ] = ValueIteration( P, G );
[ J_opt_pi, u_opt_ind_pi ] = PolicyIteration( P, G );
[ J_opt_lp, u_opt_ind_lp ] = LinearProgramming( P, G );
% Here we solve the stochastic shortest path problem by Value Iteration,
% Policy Iteration, and Linear Programming.

%% plot results
figH = PlotMaze( 2, mazeSize, walls, targetCell, [], [], stateSpace, ...
   controlSpace, J_opt_vi, u_opt_ind_vi );
figure(figH);
title(strcat('Value iteration (width=', num2str(mazeSize(1)), ', height=', num2str(mazeSize(2)), ')'));

figH = PlotMaze( 3, mazeSize, walls, targetCell, [], [], stateSpace, ...
    controlSpace, J_opt_pi, u_opt_ind_pi );
figure(figH);
title(strcat('Policy iteration (width=', num2str(mazeSize(1)), ', height=', num2str(mazeSize(2)), ')'));

figH = PlotMaze( 4, mazeSize, walls, targetCell, [], [], stateSpace, ...
    controlSpace, J_opt_lp, u_opt_ind_lp );
figure(figH);
title(strcat('Linear programming (width=', num2str(mazeSize(1)), ', height=', num2str(mazeSize(2)), ')'));

% This plots the results for all three algorithms.

%% display that terminated
disp('terminated');
