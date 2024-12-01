function [Aeq_period, beq_period] = get_periodicity_constraints(n, nu, N)
%returns periodicity constraints for QP solver
% inputs original state dimension n
%        lifted state dimension nz
%        number of inputs nu
%        number of time points N
Aeq_period = zeros(n, (n + nu)*N);
Aeq_period(:, 1:n) = eye(n);
Aeq_period(:, n*N-(n-n)-n+1:n*N-(n-n)) = -eye(n);
beq_period = zeros(n, 1);
end

