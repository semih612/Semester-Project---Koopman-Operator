function [PHI] = NLDyna_Obs_Fun144(X)
%Compute all observation functions for model 4
Ntraj = size(X,2);
PHI = zeros(2,Ntraj);
for i=1:Ntraj
    PHI(1,i) = sin(X(3,i))^2*X(2,i);
    PHI(2,i) = sin(X(1,i))^2*cos(X(1,i));
end
end