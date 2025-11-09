function [A, B, C] = calculate_A_B_C(x, u, Ntraj, Obs_state, n, m)
% CALCULATE_A_B_C Computes the Koopman-based system matrices A, B, and C.
%
% This function estimates the lifted system dynamics using a Koopman-based
% approach. It constructs the state transition matrices A and B and the
% observation matrix C from trajectory data

% Dimensions
nObs_state = size(Obs_state(zeros(n,1)));
Nlift = nObs_state;
nZ = Nlift(1);   % lifted state dimension

X = reshape(x(:, 1:Ntraj, :), n, []);
Y = reshape(x(:, 2:Ntraj+1, :), n, []);
U = reshape(u(:, 1:Ntraj, :), m, []);
% Compute lifted states
X_lift = [];
Y_lift = [];
for i = 1:size(X, 2)
    X_lift(:, i) = Obs_state(X(:, i));
    Y_lift(:, i) = Obs_state(Y(:, i));
end

% Compute A and B
Z = [X_lift; U]; % Stack lifted states and inputs
AB = Y_lift * pinv(Z);

% Compute A and B, only first four states
%AB_temp = Y_lift(1:4,:) * pinv(Z);
%C_temp = [eye(4), zeros(4,nZ-4)];
%AB = pinv(C_temp) * AB_temp;


A = AB(:, 1:size(X_lift, 1));
B = AB(:, size(X_lift, 1)+1:end);
C = X * pinv(X_lift);
save('A_koop.mat', 'A');
save('B_koop.mat', 'B');
save('C_koop.mat', 'C');

end
