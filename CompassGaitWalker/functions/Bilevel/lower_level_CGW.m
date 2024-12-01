function [Z_EDMD, U_EDMD, cost] = lower_level_CGW(L_0, L_u, C_EDMD, x0, N_opt, T, param)
%% initialize optimization
param_vec = [param.a, param.b, param.m, param.m_h, param.l, param.g];
tspan = linspace(0, T, N_opt);
dt = tspan(2);
N = length(tspan);
z0 = CGW_lift(x0);
nz = length(z0);
nu = 1;
n = 4;
[K_0, B] = cont2disc(L_0, L_u*z0, dt);
qT = [0, 1; 1, 0]*x0(1:2);
linear_jumpmap = jumpmap_lin(qT, param_vec);
linear_jumpmap_x = [0, 1, 0, 0; 1, 0, 0, 0; zeros(2), linear_jumpmap];
%% optimization problem setup

Z = optimvar('Z', [nz,N]);
U = optimvar('U', [1, N]);
qprob = optimproblem;
qprob.Objective = U*U'*dt;


dynamics_constraint = Z(:, 2:end) == K_0*Z(:, 1:end-1) + B*U(1:end-1);

qprob.Constraints.IC = Z(:, 1) == z0;
qprob.Constraints.periodicity = linear_jumpmap_x * C_EDMD*Z(:, end) == x0;
qprob.Constraints.Dynamics = dynamics_constraint;

% tic 
options = optimoptions('quadprog', 'Algorithm', 'interior-point-convex', 'Display', 'off');
[sol_struct, cost] = solve(qprob,'Options', options);
% toc
Z_EDMD = sol_struct.Z;
U_EDMD = sol_struct.U;
end

