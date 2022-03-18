classdef MPC_Control_x < MPC_Control
    
    methods
        % Design a YALMIP optimizer object that takes a steady-state state
        % and input (xs, us) and returns a control input
        function ctrl_opti = setup_controller(mpc, Ts, H)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   X(:,1)       - initial state (estimate)
            %   x_ref, u_ref - reference state/input
            % OUTPUTS
            %   U(:,1)       - input to apply to the system
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            N = ceil(H/Ts); % Horizon steps

            [nx, nu] = size(mpc.B);
            
            % Targets (Ignore this before Todo 3.2)
            x_ref = sdpvar(nx, 1);
            u_ref = sdpvar(nu, 1);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            
            % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D are
            %       the DISCRETE-TIME MODEL of your system
            
            % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE
            obj = 0;
            con = [];
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %--------------------------------------------------------------------------
            % Constraints
            % d2 in U = { d2 | |d2| <= 15deg }
            % d2 in U = { d2 | Md2 <= m }
            M = [1;-1]; m = [deg2rad(15); deg2rad(15)];
            % x in X = { x | |beta| <= 5deg}
            % x in X = { x | Fx <= f }
            F = [0 1 0 0; 0 -1 0 0]; f = [deg2rad(5); deg2rad(5)]

            %Compute a terminal controller, weight and set that will ensure recursive
            %feasibility of the closed-loop system
            %Compute the sets and weights using your code from last week, then repeat
            %the procedure to validate your results using MPT3
            
            %stage cost
            %x'Qx + u'Ru
            Q = 1*eye(nx);
            R = 1;
            
            A=mpc.A
            B=mpc.B

            % Compute LQR controller for unconstrained system
            [K,Qf,~] = dlqr(A,B,Q,R);
            % MATLAB defines K as -K, so invert its signal
            K = -K; 
            
            % Compute maximal invariant set
            Xf = polytope([F;M*K],[f;m]);
            Acl = [A+B*K];
            i=1;
            while 1
                prevXf = Xf;
                [T,t] = double(Xf);
                preXf = polytope(T*Acl,t);
                Xf = intersect(Xf, preXf);
                
                h2 = Xf.projection(1:2).plot('color', 'yellow');
                %fprintf('Iteration %i... not yet equal\n', i);
                %pause;
                if isequal(prevXf, Xf)
                    break
                end
                i=i+1;
            end
            fprintf('Maximal invariant set computed after %i iterations\n\n', i);
            [Ff,ff] = double(Xf);
                


            %PLOTS
            Xf.projection(1:2).plot('color','green');
            pause;
            %Xf.projection(2:3).plot('color','red');
            %pause;

            %--------------------------------------------------------------------------
            %use YALMIP to compute the MPC controller
            % Predicted state and input trajectories
            X = sdpvar(nx, N);
            U = sdpvar(nu, N-1);

            con = (X(:,2) == A*X(:,1) + B*U(:,1)) + (M*U(:,1) <= m);
            obj = U(:,1)'*R*U(:,1);
            for i = 2:N-1
                con = con + (X(:,i+1) == A*X(:,i) + B*U(:,i));
                con = con + (F*X(:,i) <= f) + (M*U(:,i) <= m);
                obj = obj + (X(:,i)-x_ref)'*Q*(X(:,i)-x_ref) + (U(:,i)-u_ref)'*R*(U(:,i)-u_ref);
            end
            con = con + (Ff*X(:,N) <= ff);
            obj = obj + (X(:,N)-x_ref)'*Qf*(X(:,N)-x_ref);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % Return YALMIP optimizer object
            ctrl_opti = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
                {X(:,1), x_ref, u_ref}, U(:,1));
        end
        
        % Design a YALMIP optimizer object that takes a position reference
        % and returns a feasible steady-state state and input (xs, us)
        function target_opti = setup_steady_state_target(mpc)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   ref    - reference to track
            % OUTPUTS
            %   xs, us - steady-state target
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            nx = size(mpc.A, 1);

            % Steady-state targets
            xs = sdpvar(nx, 1);
            us = sdpvar;
            
            % Reference position (Ignore this before Todo 3.2)
            ref = sdpvar;
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D
            obj = 0;
            con = [xs == 0, us == 0];
            
            A=mpc.A;
            B=mpc.B;
            C=mpc.C;
            

            % Constraints
            % d2 in U = { d2 | |d2| <= 15deg }
            % d2 in U = { d2 | Md2 <= m }
            M = [1;-1]; m = [deg2rad(15); deg2rad(15)];

            con = [M*us <= m, xs == A*xs + B*us, ref == C*xs];
            obj = ref - C*xs;
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Compute the steady-state target
            target_opti = optimizer(con, obj, sdpsettings('solver', 'gurobi'), ref, {xs, us});
        end
    end
end
