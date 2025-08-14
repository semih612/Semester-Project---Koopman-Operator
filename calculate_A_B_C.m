function [A, B, C] = calculate_A_B_C(x, u, Ntraj, Obs_state, n, m)
% CALCULATE_A_B_C Computes the Koopman-based system matrices A, B, and C.
%
% This function estimates the lifted system dynamics using a Koopman-based
% approach. It constructs the state transition matrices A and B and the
% observation matrix C from trajectory data.
%
% INPUTS:
%   - x        : (n x Ntraj+1 x Nexp) Matrix of state trajectories
%                * n     - Dimension of the state
%                * Ntraj - Number of time steps per trajectory
%                * Nexp  - Number of experiments
%   - u        : (m x Ntraj x Nexp) Matrix of input trajectories
%                * m     - Dimension of the input
%                * Ntraj - Number of time steps per trajectory
%                * Nexp  - Number of experiments
%   - Ntraj    : Scalar - Number of time steps per trajectory (excluding initial state)
%   - Obs_state: Function handle - Observable function that maps states to a lifted space
%   - n        : Scalar - Dimension of the original state space
%   - m        : Scalar - Dimension of the input space
%
% OUTPUTS:
%   - A        : Matrix - Estimated system transition matrix in lifted space
%   - B        : Matrix - Estimated input transition matrix in lifted space
%   - C        : Matrix - Observation matrix mapping original states to lifted states
%
% The function saves matrices A, B, and C into 'A_pend.mat', 'B_pend.mat', and 'C_pend.mat'
% for later use.


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

%lambda = 1e-4;  % Tune this based on scale of Y_lift
lambda = 1e-5 * norm(Z * Z', 'fro');
AB = Y_lift * Z' / (Z * Z' + lambda * eye(size(Z,1)));
AB = Y_lift * pinv(Z);


A = AB(:, 1:size(X_lift, 1));
B = AB(:, size(X_lift, 1)+1:end);
C = X * pinv(X_lift);
save('A_koop.mat', 'A');
save('B_koop.mat', 'B');
save('C_koop.mat', 'C');

end
