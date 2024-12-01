function [T_opt, cost] = upper_level_pendulum_soft(L_0, L_u, C_EDMD, x0, T_init, N_opt, opts, weight_soft_constraint)
% This function solves the upper-level in the bilevel optimization
% L_0, L_u: system matrices of the bilinear Koopman generator approximation
% C_EDMD: Matrix which recovers the original state from the lifted state
% x0: operating point and anchor
% T_init: initial guess for T for the optimizer
% N_opt: number of discretization points in the lower-level
% opts: options for fmincon
% weight_soft_constraint: weighting for the soft constraint between 0 and 1
% (0 meaning no soft constraint)

% T has to be non-negative
Aineq = -1;
bineq = 1e-3;
Aeq = [];
beq = [];
lb = [];
ub = [];
[T_opt_fmincon, cost] = fmincon(@(T) upper_level_cost_soft(L_0, L_u, C_EDMD, x0, T, N_opt, weight_soft_constraint), T_init, ...
    Aineq, bineq, Aeq, beq, lb, ub, [], opts);
T_opt = T_opt_fmincon(end);
end

