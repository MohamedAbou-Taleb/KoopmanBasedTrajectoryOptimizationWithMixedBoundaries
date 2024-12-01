function [Z_EDMD, U_EDMD, cost] = lower_level_pendulum(L_0, L_u, C_EDMD, x0, N_opt, T)
% This function solves the upper-level in the bilevel optimization
% L_0, L_u: system matrices of the bilinear Koopman generator approximation
% C_EDMD: Matrix which recovers the original state from the lifted state
% x0: operating point and anchor
% N_opt: number of discretization points in the lower-level
% T: period time
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
qprob.Objective = U*U'*dt;

% dynamics
dynamics_constraint = Z(:, 2:end) == K_0*Z(:, 1:end-1) + B*U(1:end-1);
qprob.Constraints.Dynamics = dynamics_constraint;

% Lift the left boundary
qprob.Constraints.IC = Z(:, 1) == z0;
qprob.Constraints.periodicity = C_EDMD*Z(:, end) == x0;

options = optimoptions('quadprog', 'Algorithm', 'interior-point-convex', 'Display', 'off');
[sol_struct, cost] = solve(qprob,'Options', options);

Z_EDMD = sol_struct.Z;
U_EDMD = sol_struct.U;

end

