function [PHI] = NLDyna_Obs_Fun79(X)
%Compute all observation functions for model 4
Ntraj = size(X,2);
PHI = zeros(2,Ntraj);
for i=1:Ntraj
    PHI(1,i) = sin(X(3,i));            % 1
    PHI(2,i) = cos(X(3,i))*X(4,i);     % 15
end
end