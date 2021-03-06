addpath(fullfile('..', 'src'));

%% TODO: This file should produce all the plots for the deliverable
clc;
clear;
close all;

Ts = 1/20; % Sample time
rocket = Rocket(Ts); 
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);
% Design MPC controller
H = 15; % Horizon length in seconds
mpc_roll = MPC_Control_z(sys_roll, Ts, H);
% Get control input
%ux = mpc_x.get_u(x)

x0 = [0 0.5]
[T, X_sub, U_sub] = rocket.simulate(sys_roll, x0, H, @mpc_roll.get_u, 0);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_roll, xs, us);
