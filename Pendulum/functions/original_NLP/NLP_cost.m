function cost = NLP_cost(X_U_T, N, n, dtau)
% cost function for the original trajectory optimization problem
% X_U: (n + n_u + 1) times N by 1 tall vector of state and input trajectory and the period; X_U = [X_vec; U_vec; T]
% N: number of discretization points
% n: dimension of the state x
% dtau: dt/T, where dt is the time step size and T is the period
U = X_U_T(n*N+1:end-1);
T = X_U_T(end);
dt = dtau*T;
cost = (U')*U*dt;
end