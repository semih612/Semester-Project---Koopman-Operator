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

Obs_state = @(x)[x(1,:); x(2,:); x(3,:); x(4,:); sin(x(1,:)); cos(x(1,:)); sin(x(1,:)); cos(x(1,:))];

nObs_state = size(Obs_state(zeros(n,1)));
Nlift = nObs_state;


%%  Selected Parameters
rng(1);  % fix seed for reproducibility
Nsim_train_per_region = 5;
n_region = 16;
Nsim_test = 320;
Ntraj_train = 400;
Ntraj_test = 400;

x_train = zeros(n,Ntraj_train+1,Nsim_train_per_region*n_region);
u_train = zeros(m,Ntraj_train,Nsim_train_per_region*n_region);
for iter = 1:n_region
    u_train(:,:,((iter-1)*Nsim_train_per_region+1):(iter*Nsim_train_per_region)) = 20*(rand(1, Ntraj_train, Nsim_train_per_region)-0.5);
    x01 = 4*pi*rand(1, 1, Nsim_train_per_region) - 2*pi;
    x02  = 10*pi*rand(1, 1, Nsim_train_per_region) - 5*pi;  
    angle_range = 2*pi/n_region;
    x03 = angle_range * rand(1, 1, Nsim_train_per_region) - pi + (iter-1)*angle_range;
    %x03 = (2*pi/n_region)*rand(1, 1, Nsim_train_per_region) -2*pi/n_region - pi + iter*2*pi/n_region;
    x04  = 10*pi*rand(1, 1, Nsim_train_per_region) - 5*pi;
        
    x0_train = zeros(n,1,Nsim_train_per_region);
    x0_train(1:4,1,:) = [x01; x02; x03; x04];
    %Ntraj = size(u_train,2);
    x_train(:, 1, ((iter-1)*Nsim_train_per_region+1):iter*Nsim_train_per_region) = x0_train;

    for i = ((iter-1)*Nsim_train_per_region+1):(iter*Nsim_train_per_region)
        %i
        j = 1;
        count_turn = 0;
        while j <= Ntraj_train
            x_train(:,j+1,i) = f_DT(x_train(:,j,i), u_train(:,j,i));
            if (x_train(3,j+1,i) >= 9*pi/8) || (x_train(3,j+1,i) <= -9*pi/8)
                u_train(:,:,i) = 20*(rand(1, Ntraj_train, 1)-0.5);
                count_turn = count_turn + 1;
                if count_turn == 100 
                    x01 = 4*pi*rand(1, 1, 1) - 2*pi;
                    x02  = 10*pi*rand(1, 1, 1) - 5*pi;  
                    x03 = (2*pi/n_region)*rand(1, 1, 1) -2*pi/n_region - pi + iter*2*pi/n_region;
                    x04  = 10*pi*rand(1, 1, 1) - 5*pi;
                    x_train(:,1,i) = [x01; x02; x03; x04];
                    count_turn = 0;
                end
                j = 0;
                %err = sprintf('Error: %d',i);
                %disp(err);
            end
            j = j + 1;
        end
    end
end
          
save('x_train.mat','x_train')
save('u_train.mat','u_train')
    

u_test = 20*(rand(1, Ntraj_test, Nsim_test)-0.5);
x01 = 4*pi*rand(1, 1, Nsim_test) - 2*pi;
x02 = 10*pi*rand(1, 1, Nsim_test) - 5*pi;
x03 = 2*pi*rand(1, 1, Nsim_test) - pi;
x04 = 10*pi*rand(1, 1, Nsim_test) - 5*pi;

x0_test = zeros(n,1,Nsim_test);
x0_test(1:4,1,:) = [x01; x02; x03; x04];

x_test = zeros(n,Ntraj_test+1,Nsim_test);
x_test(:,1,:) = x0_test;
i = 1;
while i <= Nsim_test
    i
    j = 1;
    count_turn = 0;
    while j <= Ntraj_test
        x_test(:,j+1,i) = f_DT(x_test(:,j,i), u_test(:,j,i));
        if (x_test(3,j+1,i) >= 9*pi/8) || (x_test(3,j+1,i) <= -9*pi/8)
            u_test(:,:,i) = 20*(rand(1, Ntraj_test, 1)-0.5);
            count_turn = count_turn + 1;
            if count_turn == 100 
                x01 = 4*pi*rand(1, 1, 1) - 2*pi;
                x02  = 10*pi*rand(1, 1, 1) - 5*pi;  
                x03 = 2*pi*rand(1, 1, 1) - pi;
                x04  = 10*pi*rand(1, 1, 1) - 5*pi;
                x_test(:,1,i) = [x01; x02; x03; x04];
                count_turn = 0;
            end
            j = 0;
            %err = sprintf('Error: %d',i);
            %disp(err);
        end
        j = j + 1;
    end
    i = i+1;
end

save('x_test.mat','x_test')
save('u_test.mat','u_test')
    