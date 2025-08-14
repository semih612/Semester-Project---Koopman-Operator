function [PHI] = NLDyna_Obs_Fun321(X)
%Compute all observation functions for model 4
Ntraj = size(X,2);
PHI = zeros(4,Ntraj);
for i=1:Ntraj
    PHI(1,i) = sin(X(3,i));
    PHI(2,i) = cos(X(3,i))*X(4,i);
    PHI(3,i) = sin(X(3,i))*sin(X(1,i))^2;
    PHI(4,i) = cos(X(3,i))^2*X(2,i);
end
end