function [x0_opt, T_opt, cost] = upper_level_CGW(L_0, L_u, C_EDMD, v_avg, x0_T, N_opt, param, opts)
% This function solves the upper-level in the bilevel optimization
% L_0, L_u: system matrices of the bilinear Koopman generator approximation
% C_EDMD: Matrix which recovers the original state from the lifted state
% v_avg: operating point / desired average speed
% x0_T: initial guess for x0 and T for the optimizer
% N_opt: number of discretization points in the lower-level
% opts: options for fmincon
x0_T_init = x0_T;
Aineq = [0,0,0,0,-1];
bineq = 1e-3;
Aeq = [1, 1, 0, 0, 0];
beq = 0;
lb = [];
ub = [];
[x0_T_opt_fmincon, cost] = fmincon(@(x0_T) upper_level_CGW_cost(L_0, L_u, C_EDMD, x0_T, N_opt, param), x0_T_init, ...
    Aineq, bineq, Aeq, beq, lb, ub, @(x0_T) upper_level_CGW_constraints(x0_T, v_avg), opts);
T_opt = x0_T_opt_fmincon(end);
x0_opt = x0_T_opt_fmincon(1:4);
end

