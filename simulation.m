function [psi_true_all, psi_pred_sim, RMSE_box, state_error, stopTraj] = simulation(x, u, Obs_state, A, B, C, Ntraj, Nsim, n, FigShow)

[num_lifted_states, ~] = size(A);
%prediction_error = zeros(Ntraj, Nsim);
simulation_error = zeros(num_lifted_states,Ntraj, Nsim);

stopTraj = ones(Nsim,1)*Ntraj;
state_error = zeros(n, Ntraj+1, Nsim);
x_sim = zeros(n, Ntraj+1, Nsim);
psi_true_all = [];
psi_pred_sim = [];

for sim = 1:Nsim
    % Initialize simulation with the first state
    x_sim(:,1,sim) = x(:,1,sim);
    psi_xk_sim(:,1,sim) = Obs_state(x(:,1,sim));

    for k = 1:Ntraj
        % Current lifted state and true next lifted state

        psi_xk1_true = Obs_state(x(:,k+1,sim));
        %psi_true_all = [psi_true_all, psi_xk1_true];
        

        % Simulated next lifted state
        psi_xk_sim(:,k+1,sim) = A * psi_xk_sim(:,k,sim) + B * u(:,k,sim);
        %psi_pred_sim = [psi_pred_sim, psi_xk1_pred];


        % Simulation error in lifted space
        simulation_error(:, k+1,sim) = (psi_xk1_true - psi_xk_sim(:,k+1,sim)).^2;
        x_sim(:,k+1,sim) = psi_xk_sim(1:4,k,sim);
    end
end

%mean_prediction_error = mean(prediction_error(:,:,index), 3);

%RMSE_state = sqrt(mean(mean(simulation_error, 3),2))';
%MSE_perc = sqrt(mean(mean(simulation_error_perc, 3),2))';
%MSE_box = sqrt(mean(mean(simulation_error_box, 3),2))';

for j = 4:num_lifted_states 
    RMSE(j-3) = sqrt(mean(sum(simulation_error(1:j,:,:), 1:2)));
    RMSE_perc(j-3) = sqrt(mean(sum(simulation_error_perc(1:j,:,:), 1:2)));
    RMSE_box(j-3) = sqrt(mean(sum(simulation_error_box(1:j,:,:), 1:2))); 
end

if FigShow
    close all

    figure;
    hold on;
    colors = lines(n); % Use distinct colors
    for i = 1:n
        subplot(n,1,i);
        plot(1:Ntraj, RMSE_state(i, 1:Ntraj), 'Color', colors(i, :), 'LineWidth', 2, ...
            'DisplayName', ['State ' num2str(i)]);
        xlabel('Time Step');
        ylabel(['RMSE of x_' num2str(i)]);
        title(['RMSE Over Time for State x_' num2str(i)]);
        legend('Location', 'best');
        grid on;

        % Display mean error values in a table inside the figure
         column_names = {'State Index', 'Mean RMSE'};
        mean_errors = mean(RMSE_state, 2);
        data = [(i:i)', mean_errors(i)]; % Format table data
    
        % Create the table in figure
        uitable('Data', data, ...
                'ColumnName', column_names, ...
                'RowName', [], ...
                'Units', 'normalized', ...
                'Position', [0.14, 1-0.22*i, 0.1, 0.06]); % Adjust position as needed
    end
    hold on
    
    sim = 1;
    figure;
    for i = 1:n
        subplot(n,1,i);
        plot(0:stopTraj(sim)-1, squeeze(x(i,1:stopTraj(sim),sim)), 'b', 'DisplayName', 'True');
        hold on;
        plot(0:stopTraj(sim)-1, squeeze(x_sim(i,1:stopTraj(sim),sim)), 'r--', 'DisplayName', 'Predicted');
        legend('Location','best');
        xlabel('Time Step');
        ylabel(['State ', num2str(i)]);
    end
    sgtitle('True vs Predicted Trajectories (Single Simulation)');
    hold on



%{
    figure;
    hold on;
    colors = lines(num_lifted_states); % Use distinct colors
    for i = 1:num_lifted_states
        plot(1:Ntraj, mean_prediction_error(i, :), 'Color', colors(i, :), 'LineWidth', 2, ...
            'DisplayName', ['Lifting State ' num2str(i)]);
    end
    xlabel('Time Step');
    ylabel('Prediction Error');
    title('Prediction Error Over Time for Each Lifted State');
    legend('show'); % Show legend for identification
    grid on;

    % Display mean error values in a table inside the figure
    column_names = {'Lifted State', 'Mean Error'};
    mean_errors = mean(mean_prediction_error, 2);
    data = [(1:num_lifted_states)', mean_errors]; % Format table data

    % Create the table in figure
    uitable('Data', data, ...
            'ColumnName', column_names, ...
            'RowName', [], ...
            'Units', 'normalized', ...
            'Position', [0.15, 0.7, 0.1, 0.2]); % Adjust position as needed


    
    figure;
    plot(1:Ntraj, mean(mean_prediction_error,1), 'LineWidth', 2);
    xlabel('Time Step');
    ylabel('Mean Prediction Error (Lifted Space)');
    title('Prediction Error Over Time Across Trajectories');
    hold on
    
    figure;
    plot(0:Ntraj, mean(vecnorm(state_error,2,1), 3), 'LineWidth', 2);
    xlabel('Time Step');
    ylabel('Mean State Reconstruction Error');
    title('State Reconstruction Error Over Time Across Trajectories');
    hold on
    
    sim = 8;
    figure;
    for i = 1:n
        subplot(n,1,i);
        plot(0:stopTraj(sim)-1, squeeze(x(i,1:stopTraj(sim),sim)), 'b', 'DisplayName', 'True');
        hold on;
        plot(0:stopTraj(sim)-1, squeeze(x_sim(i,1:stopTraj(sim),sim)), 'r--', 'DisplayName', 'Predicted');
        legend('Location','best');
        xlabel('Time Step');
        ylabel(['State ', num2str(i)]);
    end
    sgtitle('True vs Predicted Trajectories (Single Simulation)');
    hold on

    %}
    
    %numLiftedStates = size(psi_true_all, 1);   
    %figure;
    %for i = 1:numLiftedStates
    %    subplot(numLiftedStates, 1, i);
    %    plot(psi_true_all(i,:), psi_pred_all(i,:), 'o');
    %    xlabel('True Lifted State');
    %    ylabel('Predicted Lifted State');
    %    title(['Lifted State Dimension ', num2str(i)]);
    %end
    %hold on

end

end