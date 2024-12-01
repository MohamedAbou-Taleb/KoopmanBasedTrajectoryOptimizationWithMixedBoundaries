function [Aeq_dyn, beq_dyn] = get_dynamics_constraints(A, B, N)
%construct dynamics constraints for QP solver
% input: discrete state space matrices A and B
% length N
% initial condition z0
[nz, nu] = size(B);
eye_shift = diag(ones(N-1, 1), -1);
H = kron(eye_shift, A);
P = -kron(eye(N-1), B);
Gz = [zeros([nz, size(P,2)]); P];
Gz = [Gz, zeros(nz*N, 1)];
Aeq_dyn = [eye(size(H)) - H, Gz];
beq_dyn = zeros(size(Aeq_dyn, 1), 1);
beq_dyn = beq_dyn(nz+1:end);
Aeq_dyn = Aeq_dyn(nz+1:end, :);

end

