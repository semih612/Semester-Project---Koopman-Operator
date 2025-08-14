function [A_val, B_val] = calculate_A_B(x_train, u_train, Ntraj, Obs_state, n, m, Ts, mode)

    if strcmp(mode, 'oe')
        
        % Dimensions
        nObs_state = size(Obs_state(zeros(n,1)));
        Nlift = nObs_state;
        nZ = Nlift(1);   % lifted state dimension
        % Initial guess for A and B
        load('A_koop.mat');
        load('B_koop.mat');
        A0 = A;
        B0 = B;
        theta0 = [A0(:); B0(:)];
        
        Nsim = size(x_train, 3);
        N = size(u_train, 2);
        for i = 1:Nsim
            Phi0_all(:,i) = Obs_state(x_train(:,1,i));  % Φ₀
            for k = 1:N
                X_plus_all(:,k,i) = Obs_state(x_train(:,k+1,i));  % Φ(x₁) to Φ(x_N)
            end
        end
        U_all = u_train;
        
        % Optimization
        options = optimoptions('fminunc', 'Display', 'iter', 'MaxIterations', 20000, 'Algorithm', 'quasi-newton', 'MaxFunctionEvaluations', 2000000);
        cost_fun = @(theta) cost_func(theta, Phi0_all, U_all, X_plus_all, N, nZ, m, Nsim);
        [theta_opt, fval] = fminunc(cost_fun, theta0, options);
        
        % Reshape result
        A_oe = reshape(theta_opt(1:nZ*nZ), nZ, nZ);
        B_oe = reshape(theta_opt(nZ*nZ+1:end), nZ, m);
        %save('A_oe.mat','A_oe');
        %save('B_oe.mat','B_oe');
        A_val = A_oe;
        B_val = B_oe;
    
    else if strcmp(mode, 'iv')
            
        x_train_iv = x_train;
        
        nObs_state = size(Obs_state(zeros(n,1)));
        Nlift = nObs_state;
        nZ = Nlift(1);   % lifted state dimension

        pre_A = zeros(Nlift(1),Nlift(1));
        pre_B = zeros(Nlift(1),1);
        convergence1 = 1;
        convergence2 = 1;
        
        while convergence1 > 1e-7 || convergence2 > 1e-7
            [~, Ntraj, Nsim] = size(u_train);
            Xstates = reshape( x_train_iv(:, 1:Ntraj, :), n, [] );     % n x (Ntraj*Nsim_total)
            Ystates = reshape( x_train_iv(:, 2:Ntraj+1, :), n, [] );   % n x (Ntraj*Nsim_total)
            Uinputs = reshape( u_train(:, 1:Ntraj, :), m, [] );     % m x (Ntraj*Nsim_total)
            X = Xstates;
            Y = Ystates;
            U = Uinputs;
            
            % Compute lifted states
            X_lift = [];
            Y_lift = [];
            for i = 1:size(X, 2)
                X_lift(:, i) = Obs_state(X(:, i));
                Y_lift(:, i) = Obs_state(Y(:, i));
            end
            % Compute A and B
            M = [X_lift; U]; % Stack lifted states and inputs
            
            AB_mat = Y_lift * pinv(M);  % size n x (n+m)
            A_iv = AB_mat(:, 1:Nlift(1));
            B_iv = AB_mat(:, Nlift(1)+1:end);
            
            Z_train_iv = zeros(Nlift(1),Ntraj+1,Nsim);
            x_train_iv = zeros(n,Ntraj+1,Nsim);
            for sim = 1:Nsim
                Z_train_iv(:,1,sim) = Obs_state(x_train(:,1,sim));
                x_train_iv(:,1,sim) = Z_train_iv(1:4,1,sim);
                for traj = 1:Ntraj
                    Z_train_iv(:,traj+1,sim) = A_iv*Z_train_iv(:,traj, sim) + B_iv*u_train(1,traj,sim);
                    x_train_iv(:,traj+1,sim) = Z_train_iv(1:4, traj+1,sim);  % assuming state is first 4 entries
                end
            end
            
            convergence1 = norm(A_iv - pre_A, 'fro')
            convergence2 = norm(B_iv - pre_B, 'fro')
            pre_A = A_iv;
            pre_B = B_iv;
        end
        save('A_iv.mat', 'A_iv');
        save('B_iv.mat', 'B_iv');
        A_val = A_iv;
        B_val = B_iv;
     
    else if strcmp(mode, 'iv2')

        nObs_state = size(Obs_state(zeros(n,1)));
        Nlift = nObs_state;
        nZ = Nlift(1);   % lifted state dimension

        pre_A = zeros(Nlift(1),Nlift(1));
        pre_B = zeros(Nlift(1),1);
        convergence1 = 1;
        convergence2 = 1;

        [~, Ntraj, Nsim] = size(u_train);
        Xstates = reshape( x_train(:, 1:Ntraj, :), n, [] );     % n x (Ntraj*Nsim_total)
        Ystates = reshape( x_train(:, 2:Ntraj+1, :), n, [] );   % n x (Ntraj*Nsim_total)
        Uinputs = reshape( u_train(:, 1:Ntraj, :), m, [] );     % m x (Ntraj*Nsim_total)
        X = Xstates;
        Y = Ystates;
        U = Uinputs;

        % Compute lifted states
        X_lift = [];
        Y_lift = [];
        for i = 1:size(X, 2)
            X_lift(:, i) = Obs_state(X(:, i));
            Y_lift(:, i) = Obs_state(Y(:, i));
        end
        
        
        while convergence1 > 1e-7

            % Compute A and B
            M = [X_lift; U]; % Stack lifted states and inputs
            AB_mat = Y_lift * pinv(M);  % size n x (n+m)
            A_iv = AB_mat(:, 1:Nlift(1));
            B_iv = AB_mat(:, Nlift(1)+1:end);
            
            Z_train_iv = zeros(Nlift(1),Ntraj+1,Nsim);
            for sim = 1:Nsim
                Z_train_iv(:,1,sim) = Obs_state(x_train(:,1,sim));
                for traj = 1:Ntraj
                    Z_train_iv(:,traj+1,sim) = A_iv*Z_train_iv(:,traj, sim) + B_iv*u_train(1,traj,sim);
                end
            end  
            X_lift = reshape( Z_train_iv(:, 2:Ntraj+1, :), nZ, [] );
            convergence1 = norm(A_iv - pre_A, 'fro')
            convergence2 = norm(B_iv - pre_B, 'fro')
            pre_A = A_iv;
            pre_B = B_iv;
        end
        save('A_iv.mat', 'A_iv');
        save('B_iv.mat', 'B_iv');
        A_val = A_iv;
        B_val = B_iv;
    
    else if strcmp(mode, 'taylor')
        %Rotary Arm
        mr = 0.095;  %[kg]
        r = 0.085;  %[m]
        Dr = 1e-3; %[Nms/rad]
        Jr = 1/3*mr*r^2; %[kgm^2]
        
        %Pendulum
        mp = 0.024;  %[kg]
        Lp = 0.129;   %[m]
        l = Lp/2;
        Dp = 5e-5; %[Nms/rad]
        Jp = 1/3*mp*Lp^2; %[kgm^2]
        
        %DC Motor
        Rm = 8.4;    %[ohm]
        kt = 0.042;  %[Nm/A]
        km = 0.042;  %[Vs/rad]
        
        %% Equations of motion
        %Constants
        g = 9.80665;    
        c1 = mp*l*r;
        c2 = mp*g*l;
        syms x1 x2 x3 x4 u real
        x = [x1; x2; x3; x4];
        
        % Define intermediate expressions symbolically
        f1 = Jr + Jp*sin(x3)^2;
        f2 = cos(x3);
        f3 = sin(x3)*cos(x3)*x2*x4;
        f4 = sin(x3)*x4^2;
        f5 = sin(x3)*cos(x3)*x2^2;
        f6 = sin(x3);
        f7 = 1/(f1 - (c1*f2)^2/Jp);
        tauM = km/Rm*(u - km*x2);
        
        % Define nonlinear dynamics
        f_Theta_dd = f7*(tauM - Dr*x2 - c1/Jp*f2*(Jp*f5 - c2*f6 - Dp*x4) - 2*Jp*f3 + c1*f4);
        f_Alpha_dd = 1/Jp*(-c1*f2*f_Theta_dd + Jp*f5 - c2*f6 - Dp*x4);
        f_CT_sym = [x2;
                    f_Theta_dd;
                    x4;
                    f_Alpha_dd];
        
        A_jac = jacobian(f_CT_sym, x); % df/dx
        B_jac = jacobian(f_CT_sym, u); % df/du
        
        A_lin = double(subs(A_jac, [x1 x2 x3 x4 u], [0 0 0 0 0]));
        B_lin = double(subs(B_jac, [x1 x2 x3 x4 u], [0 0 0 0 0]));
        
        sys_c = ss(A_lin, B_lin, eye(4), zeros(4,1));
        sys_d = c2d(sys_c, Ts);
        Ad = sys_d.A;
        Bd = sys_d.B;
        save('A_taylor.mat','Ad');
        save('B_taylor.mat','Bd');
        A_val = Ad;
        B_val = Bd;
    else
        A_val = 0;
        B_val = 0;
    end

    end
    end

end
