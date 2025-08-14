function [PHI] = NLDyna_Obs_Fun128(X)
%Compute all observation functions for model 4
Ntraj = size(X,2);
PHI = zeros(2,Ntraj);
for i=1:Ntraj
    PHI(1,i) = cos(X(3,i))*X(4,i);              % 15
    PHI(2,i) = sin(X(1,i))*cos(X(1,i))^2;       % 64
end
end