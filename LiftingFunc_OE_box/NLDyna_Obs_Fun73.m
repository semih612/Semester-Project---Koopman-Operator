function [PHI] = NLDyna_Obs_Fun73(X)
%Compute all observation functions for model 4
Ntraj = size(X,2);
PHI = zeros(1,Ntraj);
for i=1:Ntraj
    PHI(1,i) = cos(X(1,i))*X(2,i)*X(4,i);
end
end