function [Z_EDMD, U_EDMD, cost] = lower_level_pendulum_soft(L_0, L_u, C_EDMD, x0, N_opt, T, weight_soft_constraint)
% This function solves the upper-level in the bilevel optimization
% L_0, L_u: system matrices of the bilinear Koopman generator approximation
% C_EDMD: Matrix which recovers the original state from the lifted state
% x0: operating point and anchor
% N_opt: number of discretization points in the lower-level
% T: period time
% weight_soft_constraint: weighting for the soft constraint between 0 and 1
% (0 meaning no soft constraint)
%% initialize optimization
tspan = linspace(0, T, N_opt);
dt = tspan(2);
N = length(tspan);
z0 = pendulum_lift(x0);
nz = length(z0);

% discretize the linearized dynamics
[K_0, B] = cont2disc(L_0, L_u*z0, dt);
%% optimization problem setup

Z = optimvar('Z', [nz,N]);
U = optimvar('U', [1, N]);
qprob = optimproblem;

% Cost function
w = weight_soft_constraint;
qprob.Objective = (1-w) * U*U'*dt + w * ( (Z(:, 1) - z0)' * (Z(:, 1) - z0) + (Z(:, end) - z0)' * (Z(:, end) - z0) );

% dynamics
dynamics_constraint = Z(:, 2:end) == K_0*Z(:, 1:end-1) + B*U(1:end-1);
qprob.Constraints.Dynamics = dynamics_constraint;

% No lifting on the boundaries
qprob.Constraints.IC = C_EDMD * Z(:, 1) == x0;
qprob.Constraints.periodicity = C_EDMD * Z(:, end) == x0;

options = optimoptions('quadprog', 'Algorithm', 'interior-point-convex', 'Display', 'off');
[sol_struct, cost] = solve(qprob,'Options', options);

Z_EDMD = sol_struct.Z;
U_EDMD = sol_struct.U;

end

