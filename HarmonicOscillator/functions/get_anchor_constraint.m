function [Aeq_anchor, beq_anchor] = get_anchor_constraint(anchor, n, nu, N)
%returns anchor constraint
Aeq_anchor = zeros(1, (n+nu)*N);
Aeq_anchor(:, n) = 1;
beq_anchor = anchor;
end

