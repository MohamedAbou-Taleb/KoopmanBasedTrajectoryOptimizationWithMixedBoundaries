% Harmonic Oscillator example for bilevel optimization
clear all
close all
addpath(genpath([pwd, '/../utilities']))
addpath(genpath(pwd))

%% initialize parameters

k = 1;
m = 1;
d = 0.2;
param.m = m;
param.d = d;
param.k = k;
omega0 = sqrt(k/m);
omega = omega0*sqrt(1-(d/m)^2);
T0 = 2*pi/omega0;

%% QP parameters
% number of discretization points
N = 200;

%% Solve bilevel optimization problem
T_init = 2*T0;
operating_point = 30*pi/180;
lb = 0.7*T0;
ub = 5*T0;

% solve upper-level to find the optimal period
[T_opt, cost_opt] = fmincon(@(T) upper_level_cost(operating_point, T, N, param), ...
    T_init, [], [], [], [], lb, ub);
[X_local, ~, ~] = lower_level(operating_point, T_opt, N, param);
%% Use global optimizer
% requires Global optimization toolbox
ms = MultiStart();
problem_ms = createOptimProblem('fmincon', 'objective',...
    @(T)upper_level_cost(operating_point, T, N, param), 'x0', 2*T_init, 'lb', 0.8*T0, 'ub', 5*T0);
[T_opt_ms, cost_opt_ms] = run(ms, problem_ms, 5);
[X_global, ~, ~] = lower_level(operating_point, T_opt_ms, N, param);
%%
open costOverT.fig
hold on
scatter(T_opt/T0, cost_opt, 30, 'filled', 'yellow')
scatter(T_opt_ms/T0, cost_opt_ms, 30, 'filled', 'red')
legend({'c^*', 'Local Optimizer', 'Global Optimizer'})

%% phase space
lw = 1.5;
fontsize = 16;
figure()
hold on
plot(X_local(1,:)*180/pi, X_local(2,:)*180/pi, LineWidth = lw, Color='yellow')
plot(X_global(1,:)*180/pi, X_global(2,:)*180/pi, LineWidth = lw, Color='red')
grid on
xlabel('$q$', Interpreter='latex', FontSize=fontsize)
ylabel('$\dot{q}$', Interpreter='latex', FontSize=fontsize)
legend({'Local Optimizer', 'Global Optimizer'})