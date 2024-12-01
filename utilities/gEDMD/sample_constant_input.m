function [X_lift, X_lie_derivative] = sample_constant_input(q_max, qdot_max, u, N, Lie_derivative, lifting_fcn)
% collects samples of the system using constant inputs
% generator_flag specifies whether integration outputs or lie derivative
% data is desired
% Lie_derivative: @(t,x,u) function handle of the Lie derivative

% uniformly sample the state space
n = length(q_max);
q0_vec = 2*q_max.*rand([n, N]) - q_max;
qdot0_vec = 2*qdot_max.*rand([n, N]) - qdot_max;
X = [q0_vec; qdot0_vec];

% apply lift
X_lift = lifting_fcn(X);

X_lie_derivative = zeros(size(X_lift));
U = u*ones(1,N);

% compute the lie derivative for all samples
for i = 1:N
    X_lie_derivative(:, i) = Lie_derivative(X(:,i), U(:,i));
end

end

