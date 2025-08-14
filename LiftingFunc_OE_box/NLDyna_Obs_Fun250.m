function [PHI] = NLDyna_Obs_Fun250(X)
%Compute all observation functions for model 4
Ntraj = size(X,2);
PHI = zeros(2,Ntraj);
for i=1:Ntraj
    PHI(1,i) = sin(X(3,i))*cos(X(3,i));        % 6
    PHI(2,i) = sin(X(3,i))*cos(X(1,i))^2;      % 41
end
end