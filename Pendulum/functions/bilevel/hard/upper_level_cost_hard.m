function cost = upper_level_cost_hard(L_0, L_u, C_EDMD, x0, T, N_opt)
% This function solves the lower-level problem
% L_0, L_u: system matrices of the bilinear Koopman generator approximation
% C_EDMD: Matrix which recovers the original state from the lifted state
% x0: operating point and anchor
% T: period time
% N_opt: number of discretization points in the lower-level
    [~, U, ~] = lower_level_pendulum_hard(L_0, L_u, C_EDMD, x0, N_opt, T);
    dt = T/(N_opt-1);
    cost = U*U'*dt;
end