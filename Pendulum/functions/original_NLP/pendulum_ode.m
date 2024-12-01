function x_dot = pendulum_ode(t, x, u, param)
% ode function for the mathematical pendulum
% t: time
% x: state
% u: input
% param: struct with system parameters
m = param.m;
g = param.g;
l = param.l;
d = param.d;

x_dot = [x(2);
        -g/l * sin(x(1)) - d/(m*l^2) * x(2) + 1/(m*l^2) * u];
end

