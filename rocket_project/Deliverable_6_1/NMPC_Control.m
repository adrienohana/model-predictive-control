function opti_eval = NMPC_Control(rocket, H)

import casadi.*
opti = casadi.Opti(); % Optimization problem

N = ceil(H/rocket.Ts); % MPC horizon
nx = 12; % Number of states
nu = 4;  % Number of inputs

% Decision variables (symbolic)
X_sym = opti.variable(nx, N); % state trajectory (.........vx vy vz x y z)
U_sym = opti.variable(nu, N-1); % control trajectory) (d1,d2,Pavg,Pdiff)

% Parameters (symbolic)
x0_sym  = opti.parameter(nx, 1);  % initial state
ref_sym = opti.parameter(4, 1);   % target position

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%set integration period and define discretization
h = rocket.Ts;
f = @(x,u) rocket.f(x,u);
f_discrete = @(x,u) RK4(x,u,h,f);

%ref with state shape
X_ref = [0 0 0 0 0 ref_sym(4) 0 0 0 ref_sym(1) ref_sym(2) ref_sym(3)]';

%steady state input
u_ss = [0 0 56.67 0]';

% Initial conditions
opti.subject_to(X_sym(:,1)== x0_sym);

%LQR weights

%Tuned weights for roll_max=50deg
%Q = diag([40 40 1 1 1 100 5 5 5 100 100 100]);
%R = diag([0.1 0.1 0.2 0.1]);

%Tuned weights for roll_max=15deg
Q = diag([5 5 1 1 1 100 5 5 5 100 100 100]);
R = diag([0.1 0.1 0.2 0.1]);

%Linearize system around steady state
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
sys_d = c2d(sys, rocket.Ts);

%Compute LQR controller for unconstrained system
[K,Qf,~] = dlqr(sys_d.A,sys_d.B,Q,R);

%Loss function to minimize
obj = (U_sym(:,1)-us)'*R*(U_sym(:,1)-us);
for i = 2:N-1
    obj = obj + (X_sym(:,i)-X_ref)'*Q*(X_sym(:,i)-X_ref) + (U_sym(:,i)-us)'*R*(U_sym(:,i)-us);
end
obj = obj + (X_sym(:,N)-X_ref)'*Qf*(X_sym(:,N)-X_ref);
opti.minimize(obj)


%Discretized system and input/state constraints
for i=2:N
    %System definition
    opti.subject_to(X_sym(:,i) == f_discrete(X_sym(:,i-1), U_sym(:,i-1)));

    %Input constraints
    opti.subject_to(-deg2rad(15) <= U_sym(1,i-1) <= deg2rad(15));
    opti.subject_to(-deg2rad(15) <= U_sym(2,i-1) <= deg2rad(15));
    opti.subject_to(50 <= U_sym(3,i-1) <= 80);
    opti.subject_to(-20 <= U_sym(4,i-1) <= 20);
    
    %State constraints
    opti.subject_to(-deg2rad(85) <= X_sym(5,i-1) <=deg2rad(85));
end
% ---- Setup solver ------
ops = struct('ipopt', struct('print_level', 0, 'tol', 1e-3), 'print_time', false);
opti.solver('ipopt', ops);

% Create function to solve and evaluate opti
opti_eval = @(x0_, ref_) solve(x0_, ref_, opti, x0_sym, ref_sym, U_sym);
end

function u = solve(x0, ref, opti, x0_sym, ref_sym, U_sym)

% ---- Set the initial state and reference ----
opti.set_value(x0_sym, x0);
opti.set_value(ref_sym, ref);

% ---- Solve the optimization problem ----
sol = opti.solve();
assert(sol.stats.success == 1, 'Error computing optimal input');

u = opti.value(U_sym(:,1));

% Use the current solution to speed up the next optimization
opti.set_initial(sol.value_variables());
opti.set_initial(opti.lam_g, sol.value(opti.lam_g));
end


function [x_next] = RK4(X,U,h,f)
%
% Inputs : 
%    X, U current state and input
%    h    sample period
%    f    continuous time dynamics f(x,u)
% Returns
%    State h seconds in the future
%

% Runge-Kutta 4 integration
% write your function here
   k1 = f(X,         U);
   k2 = f(X+h/2*k1, U);
   k3 = f(X+h/2*k2, U);
   k4 = f(X+h*k3,   U);
   x_next = X + h/6*(k1+2*k2+2*k3+k4);
end