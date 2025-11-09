function cost = cost_func(theta, Phi0_all, U_all, X_plus_all, N, nZ, m, Nsim)
    % Reshape parameters
    A = reshape(theta(1:nZ*nZ), nZ, nZ);
    B = reshape(theta(nZ*nZ+1:end), nZ, m);

    total_cost = 0;

    for sim = 1:Nsim
        Phi0 = Phi0_all(:,sim);         % Initial lifted state for this sim
        U = U_all(:,1:N,sim);             % Input sequence for this sim
        X_plus = X_plus_all(:,:,sim);   % Real lifted trajectory for this sim

        % Simulate lifted state trajectory
        X_hat = zeros(nZ, N);
        X_hat(:,1) = Phi0;
        for k = 1:N-1
            X_hat(:,k+1) = A * X_hat(:,k) + B * U(:,k);
        end

        % Build regressor matrix
        Z = [X_hat(:,1:end); U(:,1:end)];
        X_pred = [A B] * Z;

        % total_cost = total_cost + sqrt(sum((X_plus(1:4,:) - X_pred(1:4,:)).^2, 1:2));  % Sum of norms
        total_cost = total_cost + sum((X_plus(1:4,:) - X_pred(1:4,:)).^2, 1:2);
        %total_cost = total_cost + norm(weighted_diff, 2)^2;

    end

    cost = total_cost;
end