function [cineq, ceq] = nonlinear_constraints_NLP(Z, v_avg, dtau, ode, param)
% constraint function for the original trajectory optimization problem 
% X_U: (n + n_u + 1) times N by 1 tall vector of state and input trajectory and the period; X_U = [X_vec; U_vec; T]
% v_avg: operating point / desired average speed
% dtau: dt/T, where dt is the time step size and T is the period
% ode: @(t, x, u) ode function handle; right hand side of the continuous-time ode 
n = 4;
N = (length(Z)-1) / (n+1);
X = Z(1:N*n);
U = Z(N*n+1:end-1);
T = Z(end);
X_reshape = reshape(X, [n, N]);
X_sim = zeros(size(X_reshape));
X_sim(:, 1) = X_reshape(:,1);

dt = dtau*T;
% dynamics
for i = 2:N
    xk = X_reshape(:, i-1);
    uk = U(i-1);
    xkPlusOne = RK4(xk, uk, dt, ode);
    X_sim(:, i) = xkPlusOne;
end
sim_residue = X_sim -X_reshape;
sim_residue = sim_residue(:,2:end);
sim_residue = reshape(sim_residue, numel(sim_residue), []);
periodicity_constraint = X(1:n) - jump_map(X_reshape(:, end), param);
anchor_constraint = event_fcn(X_reshape(:,1), param);
operating_point = -2*sin(X_reshape(2, end)+param.gamma) - v_avg*T;
ceq = [sim_residue; periodicity_constraint; anchor_constraint; operating_point];


cineq = [];
end

