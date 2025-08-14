function [PHI] = NLDyna_Obs_Fun15(X)
%Compute all observation functions for model 4
Ntraj = size(X,2);
PHI = zeros(1,Ntraj);
for i=1:Ntraj
    PHI(1,i) = cos(X(3,i))*X(4,i);
end
end