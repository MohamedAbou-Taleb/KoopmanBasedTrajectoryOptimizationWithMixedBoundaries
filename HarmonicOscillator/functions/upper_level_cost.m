function cost = upper_level_cost(operating_point, T, N, param)
% Cost function of the upper-level optimization problem
% operating_point: desired amplitude of oscillation
% T: period time
% N: number of discretization points in the lower-level
% param: struct with system parameters

% solve lower-level
[~, ~, cost] = lower_level(operating_point, T, N, param);
end

