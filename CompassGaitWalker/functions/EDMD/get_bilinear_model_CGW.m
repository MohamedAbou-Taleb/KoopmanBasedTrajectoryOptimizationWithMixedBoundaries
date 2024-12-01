function [L_0, L_u, C] = get_bilinear_model_CGW(N, sampling_bounds, param)
% identify a bilinear approximation of the Koopman generator
% N: number of samples
% sampling_bounds: struct with bounds on the states
% param: struct with system parameters

theta_swing_max = sampling_bounds.theta_swing_max;
theta_stance_max = sampling_bounds.theta_stance_max;
dtheta_swing_max = sampling_bounds.dtheta_swing_max;
dtheta_stance_max = sampling_bounds.dtheta_stance_max;
q_max = [theta_swing_max; theta_stance_max];
qdot_max = [dtheta_swing_max; dtheta_stance_max];

u_sample = 0;
[X_lift_0, Y_lift_0] = sample_constant_input(q_max, qdot_max, u_sample, N,...
    @(x, u) CGW_lift_lie_derivative(x, u, param), @(x) CGW_lift(x));

u_sample = 1;
[X_lift_1, Y_lift_1] = sample_constant_input(q_max, qdot_max, u_sample, N,...
    @(x, u) CGW_lift_lie_derivative(x, u, param), @(x) CGW_lift(x));


%% apply eDMD
threshold = 1e-7; %1e-7
% compute Koopman generator approximation
L_0 = identify_matrix(X_lift_0, Y_lift_0, threshold);
L_1 = identify_matrix(X_lift_1, Y_lift_1, threshold);

L_u = L_1-L_0;

nz = length(L_0);
C = eye(4, nz);

L_0(end,:) = zeros(1, nz);
end

