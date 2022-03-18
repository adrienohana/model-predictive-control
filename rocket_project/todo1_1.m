%TODO 1.1
clear all;
close all;
clc;
Ts = 1/20;
rocket = Rocket(Ts);

%testing getForceandMomentFromThrust
%d1 and d2 are the servo deflection angle up to +-15deg (in rad here)
d1 = 0.12;
d2 = 0.2;
%P_avg and P_diff in %, P_avg between 20% and 80%; P_diff up to +-20%
P_avg = 30;
P_diff = 20;

u = [d1,d2,P_avg,P_diff]';
[b_F, b_M] = rocket.getForceAndMomentFromThrust(u);

% testing f
% w = [wx wy wz]' -> angular velocities around the body axes
% phi = [alpha beta gamma]' -> attitude of the body frame wrtt world frame
% alpha about x_b, beta about y_b, gamma about z_b
% v = [vx vy vz]' and p = [x y z]' expressed in the world frame (v = pdot)

w = [0 0 0]';
phi = [0 0 0]';
v = [1 0 0]';
p = [0 0 0]';
% [rad/s rad m/s m]
x = [w', phi', v', p']';
x_dot = rocket.f(x,u);
%--------------------------------------------------------------------------