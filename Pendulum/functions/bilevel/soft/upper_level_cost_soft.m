function cost = upper_level_cost_soft(L_0, L_u, C_EDMD, x0, T, N_opt, weight_soft_constraint)
% This function solves the lower-level problem
% L_0, L_u: system matrices of the bilinear Koopman generator approximation
% C_EDMD: Matrix which recovers the original state from the lifted state
% x0: operating point and anchor
% T: period time
% N_opt: number of discretization points in the lower-level
% weight_soft_constraint: weighting for the soft constraint between 0 and 1
% (0 meaning no soft constraint)
    [~, U, ~] = lower_level_pendulum_soft(L_0, L_u, C_EDMD, x0, N_opt, T, weight_soft_constraint);
    dt = T/(N_opt-1);
    cost = U*U'*dt;
end