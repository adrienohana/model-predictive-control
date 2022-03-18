%TODO 2.1
clear all;
close all;
clc;

Ts = 1/20;
rocket = Rocket(Ts);

%compute trim point == steady state input pair st 0=f(xs,us)
[xs, us] = rocket.trim();
sys = rocket.linearize(xs,us);
[sys_x,sys_y,sys_z,sys_roll] = rocket.decompose(sys,xs,us);

sys_x
%sys_x -> thrust vector angle d2        to     position x
% states : wy,beta,vx,x

%sys_y -> thrust vector angle d1        to     position y
% states : wx,alpha,vy,y

%sys_z -> average throttle Pavg         to     height z
% states : vz, z

%sys_z -> differential throttle Pdiff   to     roll angle gamma
% states : wz, gamma