function [L_0, L_u, C] = get_bilinear_model_pendulum(N, sampling_bounds, param_vec)
q_max = sampling_bounds.q_max;
qdot_max = sampling_bounds.qdot_max;

u_sample = 0;
[X_lift_0, Y_lift_0] = sample_constant_input(q_max, qdot_max, u_sample, N,...
    @(x, u) symbolic_lie_derivative(x, u, param_vec), @(x) pendulum_lift(x));

u_sample = 1;
[X_lift_1, Y_lift_1] = sample_constant_input(q_max, qdot_max, u_sample, N,...
    @(x, u) symbolic_lie_derivative(x, u, param_vec), @(x) pendulum_lift(x));

% compute Koopman approximation: custom CGW dictionary
threshold = 1e-7; %1e-7


%% apply eDMD

% compute Koopman approximation: custom CGW dictionary
L_0 = identify_matrix(X_lift_0, Y_lift_0, threshold);
L_1 = identify_matrix(X_lift_1, Y_lift_1, threshold);

L_u = L_1-L_0;

nz = length(L_0);
C = eye(2, nz);
end

