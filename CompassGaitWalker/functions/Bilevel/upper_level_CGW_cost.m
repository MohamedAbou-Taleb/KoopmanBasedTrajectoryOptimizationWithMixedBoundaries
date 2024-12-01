function cost = upper_level_CGW_cost(L_0, L_u, C_EDMD, x0_T, N_opt, param)
% Cost function for the upper-level
% L_0, L_u: system matrices of the bilinear Koopman generator approximation
% C_EDMD: Matrix which recovers the original state from the lifted state
% x0_T: initial guess for x0 and T for the optimizer
% N_opt: number of discretization points in the lower-level
    x0 = x0_T(1:4);
    T = x0_T(end);
    [~, ~, cost] = lower_level_CGW(L_0, L_u, C_EDMD, x0, N_opt, T, param);
end