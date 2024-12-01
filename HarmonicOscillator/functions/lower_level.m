function [X, U, cost_QP] = lower_level(operating_point, T, N, param)
A = [0, 1; -param.k/param.m, -param.d/param.m];
B = [0; 1/param.m];
[n, nu] = size(B);
x0 = [operating_point; 0];
%% Initialize QP
dtau = 1/(N-1);
[A_discrete, B_discrete] = cont2disc(A*T, B*T, dtau); %scale dynamics
%%%% cost function %%%%%%%
quad_cost_matrix = zeros(N*(n+nu)); % quadratic cost
quad_cost_matrix(n*N+1:end, n*N+1:end) = eye(nu*N);
linear_cost = zeros(N*(n+nu),1)';

fprintf('\n Defining QP constraints... \n')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Dynamics constraints %%%%%%%%%
fprintf('\n... dynamics constraints ...\n')

[Aeq_dyn, beq_dyn] = get_dynamics_constraints(A_discrete, B_discrete, N);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Operating Point %%%%%%%%%%%%%% ????????
fprintf('\n ... Operating point ... \n')

[Aeq_op, beq_op] = get_operating_point_constraints(x0(1), n, nu, N);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Anchor %%%%%%%%%%%%%%%%%%%%%%%
fprintf('\n ... Anchor ... \n')
[Aeq_anchor, beq_anchor] = get_anchor_constraint(x0(2), n, nu, N);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Periodicity %%%%%%%%%%%%%%%%%%
fprintf('\n ... periodicity ... \n')

[Aeq_period, beq_period] = get_periodicity_constraints(n, nu, N);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Aeq = [Aeq_dyn; Aeq_op; Aeq_anchor; Aeq_period];
beq = [beq_dyn; beq_op; beq_anchor; beq_period];
%%%%%% Inequality constraints %%%%%%
Aineq = [];
bineq = [];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf('\n ... done \n')
%% solve QP
fprintf('\n Solve QP ... \n')
maxiter = 1000; % 1000
options = optimoptions('quadprog','Display','iter', 'MaxIterations', maxiter, 'ConstraintTolerance',1e-10);
lb = [];
ub = [];

XI_QP = quadprog(quad_cost_matrix, linear_cost, Aineq, bineq, Aeq, beq, lb, ub, [], options);
fprintf('\n ... done \n')
X = XI_QP(1:n*N);
X = reshape(X, [n, N]);
U = XI_QP(n*N+1:end);

%% compute cost
dt = dtau*T;
cost_QP = (0.5*(U')*U)*dt;
disp(['Cost QP: ', num2str(cost_QP)])
end

