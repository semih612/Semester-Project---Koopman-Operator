function [PHI] = NLDyna_Obs_Fun29(X)
%Compute all observation functions for model 4
Ntraj = size(X,2);
PHI = zeros(1,Ntraj);
for i=1:Ntraj
    PHI(1,i) = sin(X(3,i))^2*cos(X(1,i));
end
end