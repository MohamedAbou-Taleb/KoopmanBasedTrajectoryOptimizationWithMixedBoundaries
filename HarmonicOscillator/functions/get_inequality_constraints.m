function [Aineq, bineq] = get_inequality_constraints(n, nz, nu, N, T_min, T_max)
%GET_INEQUALITY_CONSTRAINTS Summary of this function goes here
%   Detailed explanation goes here
Aineq = zeros(N, (nz+nu)*N);
T_inds = (n+1:nz:N*(nz));
for i = 1:length(T_inds)
    Aineq(i, T_inds(i)) = -1;
end
bineq = T_min.*ones(N, 1);
Aineq = [Aineq; -Aineq];
bineq = [bineq; T_max.*ones(N,1)];
end

