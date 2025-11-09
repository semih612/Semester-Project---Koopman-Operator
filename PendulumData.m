function [x_train, u_train, x_test, u_test] = PendulumData(mode)

    if strcmp(mode, 'load')
        d1 = load('x_train.mat');
        x_train = d1.x_train;
        d2 = load('u_train.mat');
        u_train = d2.u_train;
        d3 = load('x_test.mat');
        x_test = d3.x_test;
        d4 = load('u_test.mat');
        u_test = d4.u_test;

    else if strcmp(mode, 'test_and_produce')
    
        addpath('C:\Users\semihzaman\OneDrive - sabanciuniv.edu\Desktop\SemesterProject\LiftingFunc');
        Ts = 0.01;
        %Ts = 0.04;
        %System Caracteristics
        n = 4;     %Number of states
        m = 1;     %Number of inputs
        p = 1;     %Number of outputs
        [f_CT, f_DT] = qubeServo2Dynamics(Ts);
        
        Obs_state = @(x)[x(1,:); x(2,:); x(3,:); x(4,:); NLDyna_Obs_Fun300(x)];
        %Obs_state_1 = @(x)[x(1,:); x(2,:); x(3,:); x(4,:)];
        
        nObs_state = size(Obs_state(zeros(n,1)));
        Nlift = nObs_state;
        
        
        %% Trajectory Generation for computing the models through EDMD
        
        rng(12347);  % fix seed for reproducibility
        Nsim_train_per_region = 5;
        n_region = 16;
        Nsim_test = 320;
        Ntraj_train = 100;
        Ntraj_test = 1000;
        
        %% Test Data
        u_test = 20*(rand(1, Ntraj_test, Nsim_test)-0.5);
        x01 = 4*pi*rand(1, 1, Nsim_test) - 2*pi;
        x02 = 10*pi*rand(1, 1, Nsim_test) - 5*pi;
        x03 = 2*pi*rand(1, 1, Nsim_test) - pi;
        x04 = 10*pi*rand(1, 1, Nsim_test) - 5*pi;
        
        x0_test = zeros(n,1,Nsim_test);
        x0_test(1:4,1,:) = [x01; x02; x03; x04];
        
        %Ntraj = size(u_test,2);
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
        
        %% Train Data
        
        count = 1;
        RMSE = zeros(1,10);
        while count ~= 11
            sum = 0;
            for iter = 1:10
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
            
                [A, B, C] = calculate_A_B_C(x_train, u_train, Ntraj_train, Obs_state, n, m);
                %Ad = eye(8) + Ts*A + (Ts^2)/2*(A^2) + (Ts^3)/6*(A^3) + (Ts^4)/24*(A^4)
                %norm(Ad)
                [psi_true_all, psi_pred_sim, sim_err, state_error, stopTraj] = simulation(x_test, u_test, Obs_state, A, B, C, Ntraj_test, Nsim_test, n, false);
                RMSE_state = sqrt(mean(sim_err, 3));
                mean_error = mean(mean(RMSE_state, 2));
                sum = sum + mean_error;
            end
            sum = sum/10;
            RMSE(count) = sum;
            %Nsim_train_per_region = Nsim_train_per_region + 1;
            Ntraj_train = Ntraj_train + 100;
            count = count + 1;
        end
        
        %% Plotting
        
        numSeeds = 3;
        maxN = 10;
        
        figure;
        for sIdx = 1:numSeeds
            ax(sIdx) = subplot(numSeeds, 1, sIdx);  % one row per seed
            if sIdx == 1
                plot((1:maxN)*100, RMSE_1, '-o', 'LineWidth', 1.5);
            elseif sIdx == 2
                plot((1:maxN)*100, RMSE_2, '-o', 'LineWidth', 1.5);
            else
                plot((1:maxN)*100, RMSE_3, '-o', 'LineWidth', 1.5);
            end
            
            xlabel('Number of timesteps per train trajectory');
            ylabel('Avg. RMSE');
            grid on;
            % Optionally fix the y-limits to be the same across all subplots:
            % ylim([ymin, ymax]);  % set after observing data or compute global min/max
        end
        
        % Link x-axes (and optionally y-axes) so zoom/pan stays synchronized:
        linkaxes(ax, 'x');  % link x-axis
        % linkaxes(ax, 'y'); % if you also want the same y-limits
        
        % Add a common overall title:
        sgtitle('Validation RMSE vs. Training Data Size for Different Seeds');
        
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

    else
        x_train = 0; 
        u_train = 0; 
        x_test = 0; 
        u_test =0;
    end
    
end
