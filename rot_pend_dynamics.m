clc;
clear all;
close all;
addpath('C:\Users\semihzaman\OneDrive - sabanciuniv.edu\Desktop\SemesterProject\LiftingFunc');

Ts = 0.01;
%System Caracteristics
n = 4;     %Number of states
m = 1;     %Number of inputs
p = 1;     %Number of outputs
[f_CT, f_DT] = qubeServo2Dynamics(Ts);

Obs_state = @(x)[x(1,:); x(2,:); x(3,:); x(4,:); NLDyna_Obs_Fun191(x)];
%Obs_state_1 = @(x)[x(1,:); x(2,:); x(3,:); x(4,:)];

nObs_state = size(Obs_state(zeros(n,1)));
Nlift = nObs_state;

%% Load or Produce Data
[x_train, u_train, x_test, u_test] = PendulumData('load');
[~, Ntraj, Nsim_test] = size(u_test); 


%% Load A and B

if exist('A_koop.mat', 'file') == 2
    % File exists, so load the matrices
    load('A_koop.mat');
    load('B_koop.mat');
    load('C_koop.mat');
    disp('Matrices loaded from the file.');
else
    %x_train(1,:,:) = wrapToPi(x_train(1,:,:));
    [A, B, C] = calculate_A_B_C(x_train, u_train, Ntraj, Obs_state, n, m);
end

if exist('A_oe.mat', 'file') == 2
    % File exists, so load the matrices
    load('A_oe.mat');
    load('B_oe.mat');
    disp('Matrices loaded from the file.');
else
    [A, B] = calculate_A_B_oe(x_train, u_train, Ntraj, Obs_state, n, m);
end

%[psi_true_all, psi_pred_one_step, prediction_error, state_error] = one_step_prediction(x, u, Obs_state_1, A, B, C, Ntraj, Nsim, n, false);
%[psi_true_all, psi_pred_sim, prediction_error, state_error, stopTraj] = simulation(x, u, Obs_state_1, A, B, C, Ntraj, Nsim, n, false);


%%  A,B from Taylor

% Define symbolic variables
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

%% 

nTrials = 50;
horizon = 200;  % MPC horizon for simulation
u_lim = [-10, 10];

RMSE_koop = zeros(nTrials,1);
RMSE_taylor = zeros(nTrials,1);
cost_koop = zeros(nTrials,1);
cost_taylor = zeros(nTrials,1);
effort_koop = zeros(nTrials,1);
effort_taylor = zeros(nTrials,1);
A_oe = load('A_oe.mat');
A_oe = A_oe.A_oe;
B_oe = load('B_oe.mat');
B_oe = B_oe.B_oe;

for i = 1:nTrials
    fprintf("Running trial %d/%d...\n", i, nTrials);
    if rand < 0.5
        % Pick from [-π, -3]
        x01 = -pi + (pi - 3) * rand;
    else
        % Pick from [3, π]
        x01 = 3 + (pi - 3) * rand;
    end
    %x01 = 2*pi*rand(1, 1, 1) - pi;
    x02 = 10*pi*rand(1, 1, 1) - 5*pi;
    x03 = 2*pi*rand(1, 1, 1) - pi;
    x04 = 10*pi*rand(1, 1, 1) - 5*pi;
    z0 = [x01; x02; x03; x04]
    %Q = eye(size(A,1));
    Np = 30;        
    Q = zeros(size(A,1));
    %Q(1,1) = 1; Q(2,2) = 3; Q(3,3) = 3; Q(4,4) = 5; Q(5,5) = 5; Q(6,6) = 0;
    Q(1,1) = 10; Q(2,2) = 1; Q(3,3) = 1; Q(4,4) = 1; Q(5,5) = 1; Q(6,6) = 1;Q(7,7) = 1; Q(8,8) = 1;
    %Q(1,1) = 100000; Q(2,2) = 10000; Q(3,3) = 100; Q(4,4) = 10000; Q(5,5) = 1; Q(6,6) = 1;Q(7,7) = 1; Q(8,8) = 1;
    R = 0.001*eye(size(B,2));

    % Compute LQR controller for unconstrained system
    [K,P,~] = dlqr(A,B,Q,R);
    K = -K; 

    %z(:,1,1) = z0;
    %for k = 1:horizon-1
    %    Z(:,k,i) = Obs_state_1(z(:,k,i));
    %    z(:,k+1,1) = f_DT(z(:,k,1),K*Z(:,k,1));
    %end
    
    F = [eye(Nlift(1)); -eye(Nlift(1))];
    f = inf(2 * Nlift(1), 1); 
    f(3) = pi;
    f(Nlift(1)+3) = pi;

    M = [1; -1];
    m = [10; 10];
    Xf = polytope([F;M*K],[f;m]);
    Acl = A + B*K;

    for idx = 1:100
        PreH = [F;M*K] * Acl;
        PreX = polytope(PreH, [f;m]);
        % Intersection
        Xnew = intersect(Xf, PreX);
        if isequal(Xf, Xnew)
            %fprintf('Invariant set converged at iteration %d\n', i);
            break;
        end
        Xf = Xnew;
    end

    % Initialize prediction with the first state
    z(:,1,i) = z0;
    Z(:,1,i) = Obs_state_1(z0); % Initial lifted state
    for k = 1:horizon-1
        [u_mpc(1,k,i), infeasibility] = koopman_mpc(A, B, Q, R, Np, Z(:,k,i), u_lim, Xf, P);
        if infeasibility
            break
        end
        z(:,k+1,i) = f_DT(z(:,k,i), u_mpc(1,k,i));
        z(1,k+1,i) = wrapToPi(z(1,k+1,i));
        Z(:,k+1,i) = Obs_state_1(z(:,k+1,i));
        
        %Z(:,k+1,i) = A*Z(:,k,i) + B*u_mpc(1,k,i);
        %z(:,k+1,i) = Z(1:4,k+1,i);
        %Z(1,k+1,i) = wrapToPi(Z(1,k+1,i));
           
    end
    
    %plot(u_log); hold on; yline([-10 10], '--r'); title('Input vs Limits');
    %figure;
    %plot(angle_log); hold on; yline([-4*pi/5 4*pi/5], '--r'); title('Angle Constraint');

    % Metrics
    x_ref = [0; 0; 0; 0];  % or any desired setpoint
    RMSE_koop(i) = sqrt(mean(vecnorm(z(:,:,i) - x_ref, 2).^2));
    cost_koop(i) = 0;
    for k = 1:horizon-1
        xk = z(:,k,i);                  % State at time step k
        uk = u_mpc(:,k,i);                % Control input at time step k
        cost_koop(i) = cost_koop(i) + xk'*Q(1:4,1:4)*xk + uk'*R*uk;
    end

%%
    % Initialize prediction with the first state
    z_oe(:,1,i) = z0;
    Z_oe(:,1,i) = Obs_state_1(z0); % Initial lifted state
    for k = 1:horizon-1
        [u_mpc_3(1,k,i), infeasibility] = koopman_mpc(A_oe, B_oe, Q, R, Np, Z_oe(:,k,i), u_lim, Xf, P);
        if infeasibility
            break
        end
        z_oe(:,k+1,i) = f_DT(z_oe(:,k,i), u_mpc_3(1,k,i));
        z_oe(1,k+1,i) = wrapToPi(z_oe(1,k+1,i));
        Z_oe(:,k+1,i) = Obs_state_1(z_oe(:,k+1,i));
        
        %Z(:,k+1,i) = A*Z(:,k,i) + B*u_mpc(1,k,i);
        %z(:,k+1,i) = Z(1:4,k+1,i);
        %Z(1,k+1,i) = wrapToPi(Z(1,k+1,i));
           
    end

    % Metrics
    x_ref = [0; 0; 0; 0];  % or any desired setpoint
    RMSE_oe(i) = sqrt(mean(vecnorm(z_oe(:,:,i) - x_ref, 2).^2));
    cost_oe(i) = 0;
    for k = 1:horizon-1
        xk = z_oe(:,k,i);                  % State at time step k
        uk = u_mpc_3(:,k,i);                % Control input at time step k
        cost_oe(i) = cost_oe(i) + xk'*Q(1:4,1:4)*xk + uk'*R*uk;
    end

%% 

    Q = eye(size(Ad,1));
    Q(1,1) = 10; Q(3,3) = 1;
    R = 0.001*eye(size(Bd,2));
    Np=30;

    % Initialize prediction with the first state
    y0 = z0;
    y(:,1,i) = y0;
    for k = 1:horizon-1
        [u_mpc_2(1,k,i), infeasibility]= taylor_mpc(Ad, Bd, Q, R, Np, y0, u_lim, Xf, P);
        if infeasibility
            break
        end

        y(:,k+1,i) = f_DT(y(:,k,i), u_mpc_2(1,k,i));
        %y(:,k+1,i) = Ad*y(:,k,i) + Bd*u_mpc_2(1,k,i);
        y0 = y(:,k+1,i);
    end

    % Metrics
    x_ref = [0; 0; 0; 0];  % or any desired setpoint
    RMSE_taylor(i) = sqrt(mean(vecnorm(y(:,:,i) - x_ref, 2).^2));

    cost_taylor(i) = 0;
    for k = 1:horizon-1
        xk = y(:,k,i);
        uk = u_mpc_2(:,k,i);
        cost_taylor(i) = cost_taylor(i) + xk'*Q*xk + uk'*R*uk;
    end

end


%% Boxplots
figure;
subplot(3,1,1); boxplot([RMSE_koop, RMSE_taylor], 'Labels', {'Koopman','Taylor'}); ylabel('RMSE');
subplot(3,1,2); boxplot([cost_koop, cost_taylor], 'Labels', {'Koopman','Taylor'}); ylabel('Cost');
subplot(3,1,3); boxplot([effort_koop, effort_taylor], 'Labels', {'Koopman','Taylor'}); ylabel('Control Effort');
sgtitle('MPC Performance Comparison: Koopman vs Taylor');



%% 
sim = 1;
figure;
for i = 1:n
    subplot(n,1,i);
    plot(0:horizon-1, squeeze(z(i,1:horizon,sim)), 'b', 'DisplayName', 'MPC Koopman');
    hold on;
    plot(0:horizon-1, squeeze(y(i,1:horizon,sim)), 'r--', 'DisplayName', 'MPC Taylor');
    hold on;
    plot(0:horizon-1, squeeze(z_oe(i,1:horizon,sim)), 'g--', 'DisplayName', 'MPC OE');
    legend('Location','best');
    xlabel('Time Step');
    ylabel(['State ', num2str(i)]);
end
%sgtitle('True vs Predicted Trajectories (Single Simulation)');
hold on


%% 

J_koopman = 0;
J_taylor = 0;
for k = 1:Ntraj/2+1-100
    J_koopman = J_koopman + z(2:4,k)'*Q(2:4,2:4)*z(2:4,k) + u_mpc(:,k)'*R*u_mpc(:,k);
    J_taylor  = J_taylor  + y(2:4,k)'*Q(2:4,2:4)*y(2:4,k) + u_mpc_2(:,k)'*R*u_mpc_2(:,k);
end
disp(J_koopman)
disp(J_taylor)
k=1;
eps = 5e-2;  % Tolerance
settling_idx_koop = find(all(abs(z(2:4,k:end)) < eps, 1), 1);
settling_idx_taylor = find(all(abs(y(2:4,k:end)) < eps, 1), 1);
disp(settling_idx_koop)
disp(settling_idx_taylor)

MSE_koop = mean(sum(z.^2, 1));
MSE_taylor = mean(sum(y.^2, 1));
disp(MSE_koop)
disp(MSE_taylor)












%% Test-1

%{
function [PHI] = NLDyna_Obs_Fun3(X)
%Compute all observation functions for model 4
Ntraj = size(X,2);
PHI = zeros(4,Ntraj);
for i=1:Ntraj
    PHI(1,i) = sin(X(3,i))*cos(X(3,i))*X(2,i)*X(4,i);
    PHI(2,i) = sin(X(3,i))*X(4,i)^2;
    PHI(3,i) = sin(X(3,i))*cos(X(3,i))*(X(2,i))^2;
    PHI(4,i) = sin(X(3,i));
end
end

%}

%% Test-2

%{
function [PHI] = NLDyna_Obs_Fun3(X)
%Compute all observation functions for model 4
Ntraj = size(X,2);
PHI = zeros(5,Ntraj);
for i=1:Ntraj
    PHI(1,i) = sin(X(3,i))*cos(X(3,i))*X(2,i)*X(4,i);
    PHI(2,i) = sin(X(3,i))*X(4,i)^2;
    PHI(3,i) = sin(X(3,i))*cos(X(3,i))*(X(2,i))^2;
    PHI(4,i) = sin(X(3,i));
    PHI(5,i) = cos(X(3,i));
end
end
%}
