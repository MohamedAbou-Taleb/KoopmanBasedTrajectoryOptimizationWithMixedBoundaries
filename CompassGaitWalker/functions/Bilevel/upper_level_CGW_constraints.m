function [cineq, ceq] = upper_level_CGW_constraints(x0_T, v_avg)
    % Nonlinear constrains for the upper-level
    cineq = [];
    T = x0_T(end);
    theta_stance = x0_T(2);
    % operating point constraint
    ceq = 2*sin(theta_stance) - v_avg*T;
end