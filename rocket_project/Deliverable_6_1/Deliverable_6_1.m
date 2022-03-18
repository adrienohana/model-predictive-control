addpath(fullfile('..', 'src'));

%% TODO: This file should produce all the plots for the deliverable

Ts = 1/10; % Note that we choose a larger Ts here to speed up the simulation
rocket = Rocket(Ts);
H = 5
nmpc = NMPC_Control(rocket, H);
Tf = 30;

% MPC reference with default maximum roll = 15 deg
ref = @(t_, x_) rocket.MPC_ref(t_, Tf);
% MPC reference with specified maximum roll = 50 deg
%roll_max = deg2rad(50);
%ref = @(t_, x_) rocket.MPC_ref(t_, Tf, roll_max);

x0 = [0 0 0 0 0 0 0 0 0 0 0 0]';

[T, X, U, Ref] = rocket.simulate_f(x0, Tf, nmpc, ref);

% Plot pose
rocket.anim_rate = 1; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
