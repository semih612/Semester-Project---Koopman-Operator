clc;
clear all;
close all;
addpath('C:\Users\semihzaman\OneDrive - sabanciuniv.edu\Desktop\SemesterProject\LiftingFunc_OE_box');

Ts = 0.01;
%System Caracteristics
n = 4;     %Number of states
m = 1;     %Number of inputs   
p = 1;     %Number of outputs
[f_CT, f_DT] = qubeServo2Dynamics(Ts);

Obs_state = @(x)[x(1,:); x(2,:); x(3,:); x(4,:); NLDyna_Obs_Fun1(x)];
%Obs_state = @(x)[x(1,:); x(2,:); x(3,:); x(4,:)];

nObs_state = size(Obs_state(zeros(n,1)));
Nlift = nObs_state;

%% Load or Produce Data
[x_train, u_train, x_test, u_test] = PendulumData('load');
[~, Ntraj, Nsim_test] = size(u_test);
Ntraj = 300;
%figure;
%subplot(2,2,1); histogram(x_train(1,:)); title('x_1');
%subplot(2,2,2); histogram(x_train(2,:)); title('x_2');
%subplot(2,2,3); histogram(x_train(3,:)); title('x_3 (angle)');
%subplot(2,2,4); histogram(x_train(4,:)); title('x_4');

%% 
[A, B, ~] = calculate_A_B_C(x_train, u_train, Ntraj, Obs_state, n, m);
[A_iv, B_iv] = calculate_A_B(x_train, u_train, Ntraj, Obs_state, n, m, Ts, 'iv2');


%% 

%[psi_true_all, psi_pred_one_step, prediction_error, state_error] = one_step_prediction(x, u, Obs_state_1, A, B, C, Ntraju, Nsim, n, true);
[psi_true_all, psi_pred_sim, prediction_error, state_error, stopTraj] = simulation(x_test, u_test, Obs_state, A_iv, B_iv, 0, Ntraj, Nsim_test, n, false);
%openfig('C:\Users\semihzaman\OneDrive - sabanciuniv.edu\Desktop\SemesterProject\LiftingFunc\Figures_RMSE\NoLifting.fig');

% delete is necessary here %
delete('A_koop.mat');  
delete('B_koop.mat');
delete('C_koop.mat');

save('C:\Users\semihzaman\OneDrive - sabanciuniv.edu\Desktop\SemesterProject\OE\A_oe.mat','A_oe');
save('C:\Users\semihzaman\OneDrive - sabanciuniv.edu\Desktop\SemesterProject\OE\B_oe.mat','B_oe');

