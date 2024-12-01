function [Aeq_op, beq_op] = get_operating_point_constraints(operating_point, n, nu, N)
% returns operating point constraint
Aeq_op = zeros(1, (n+nu)*N);
Aeq_op(:, 1) = 1;
beq_op = operating_point;
end

