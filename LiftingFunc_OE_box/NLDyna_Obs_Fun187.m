function [PHI] = NLDyna_Obs_Fun187(X)
%Compute all observation functions for model 4
Ntraj = size(X,2);
PHI = zeros(3,Ntraj);
for i=1:Ntraj
    PHI(1,i) = sin(X(3,i));                             % 1
    PHI(2,i) = sin(X(3,i))*cos(X(3,i))*sin(X(1,i));     % 33
    PHI(3,i) = sin(X(3,i))*sin(X(1,i))*cos(X(1,i));     % 38
end
end