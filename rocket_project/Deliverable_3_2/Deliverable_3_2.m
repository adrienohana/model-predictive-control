addpath(fullfile('..', 'src'));

%% TODO: This file should produce all the plots for the deliverable
%ux = mpc_x.get_u(x, x_position_reference)
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
mpc_roll = MPC_Control_roll(sys_roll, Ts, H);
% Get control input
%ux = mpc_x.get_u(x)

x0 = [0 0];
x_ref = 0.2;
[T, X_sub, U_sub] = rocket.simulate(sys_roll, x0, H, @mpc_roll.get_u, x_ref);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_roll, xs, us, x_ref);