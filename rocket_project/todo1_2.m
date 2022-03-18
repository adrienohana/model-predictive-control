%TODO 1.2
clear all;
close all;
clc;

Ts = 1/20;
rocket = Rocket(Ts);
Tf = 2.0;
% (w, phi, v, p) Initial state
x0 = [deg2rad([0 0 0, 0 0 0]), 0 0 0, 0 0 0]';
% (d1 d2 Pavg Pdiff) Constant input
u = [deg2rad([0 0]), 57, 10 ]';
% Simulate nonlinear dynamics f
[T, X, U] = rocket.simulate_f(x0, Tf, u);
rocket.anim_rate = 1.0;
% Trajectory visualization at 1.0x real−time
rocket.vis(T, X, U); 

%hovering input : 
%u = [deg2rad([0 0]), 57, 0 ]';
%ascend input : 
%u = [deg2rad([0 0]), 60, 0 ]';
%descend input : 
%u = [deg2rad([0 0]), 50, 0 ]';
%rotate input : 
%u = [deg2rad([0 0]), 57, 10 ]';
%rotate input : 
%u = [deg2rad([0 2]), 57, 0 ]';
%rotate input : 
%u = [deg2rad([2 0]), 57, 0 ]';
%translation input
%u = [deg2rad([0.2 -0.2]), 55, 0 ]';
%translation input
%u = [deg2rad([-0.2 0.2]), 55, 0 ]';
