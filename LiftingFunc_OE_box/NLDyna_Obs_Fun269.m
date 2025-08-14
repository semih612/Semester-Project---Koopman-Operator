function [PHI] = NLDyna_Obs_Fun269(X)
%Compute all observation functions for model 4
Ntraj = size(X,2);
PHI = zeros(2,Ntraj);
for i=1:Ntraj
    PHI(1,i) = sin(X(3,i))*cos(X(3,i))*X(2,i)*X(4,i);
    PHI(2,i) = X(2,i)*X(4,i)^2;
end
end