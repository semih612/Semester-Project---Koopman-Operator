function [PHI] = NLDyna_Obs_Fun253(X)
%Compute all observation functions for model 4
Ntraj = size(X,2);
PHI = zeros(5,Ntraj);
for i=1:Ntraj
    PHI(1,i) = sin(X(3,i))*cos(X(3,i));            % 6
    PHI(2,i) = sin(X(3,i))*cos(X(1,i))^2;          % 41
    PHI(3,i) = cos(X(3,i))^2*sin(X(1,i));          % 47 
    PHI(4,i) = cos(X(3,i))*sin(X(1,i))*X(2,i);     % 53
    PHI(5,i) = cos(X(1,i))*sin(X(1,i))*X(2,i);     % 65
end
end