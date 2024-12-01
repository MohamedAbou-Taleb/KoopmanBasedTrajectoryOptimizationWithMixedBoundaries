function [cineq, ceq] = NLP_constraint(X_U_T, x0, dtau, ode)
% constraint function for the original trajectory optimization problem 
% X_U: (n + n_u + 1) times N by 1 tall vector of state and input trajectory and the period; X_U = [X_vec; U_vec; T]
% x0: operating point and anchor
% dtau: dt/T, where dt is the time step size and T is the period
% ode: @(t, x, u) ode function handle; right hand side of the continuous-time ode 
n = length(x0);
N = (length(X_U_T)-1) / (n+1);
X = X_U_T(1:N*n);
U = X_U_T(N*n+1:end-1);
T = X_U_T(end);
X_reshape = reshape(X, [n, N]);
X_sim = zeros(size(X_reshape));
X_sim(:, 1) = X_reshape(:,1);

dt = dtau*T;
for i = 2:N
    xk = X_reshape(:, i-1);
    uk = U(i-1);
    xkPlusOne = RK4(xk, uk, dt, ode);
    X_sim(:, i) = xkPlusOne;
end
sim_residue = X_sim -X_reshape;
sim_residue = sim_residue(:,2:end);
sim_residue = reshape(sim_residue, numel(sim_residue), []);
ceq = [sim_residue; X_reshape(:,1) - x0; X_reshape(:, end) - x0];

cineq = [];
end

