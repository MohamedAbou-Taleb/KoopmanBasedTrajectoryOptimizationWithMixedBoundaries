% Koopman Based Trajectory Optimization Example: Mathematical Pendulum with
% hard constraints
clear all
close all
addpath(genpath([pwd, '/../utilities']))
addpath(genpath(pwd))
%% initialize parameters

g = 1; % gravity
m = 1; % mass
l = 1; % length
d = 0.1; % damping

param.g = g;
param.m = m;
param.l = l;
param.d = d;

param_vec = [m, g, l, d];
n = 2;
%% sampling parameters
% number of samples
N = 45000;

% sampling bounds
q_max = 20*pi/180;
qdot_max = 1;

sampling_bounds.q_max = q_max;
sampling_bounds.qdot_max = qdot_max;
%% apply eDMD
% regression parameters 
threshold = 1e-7; % truncation parameter for svd

% compute Koopman approximation: custom CGW dictionary

[L_0, L_u, C] = get_bilinear_model_pendulum(N, sampling_bounds, param_vec);
nz = length(L_0);

% Operating point and anchor
amplitude = 30;
x0 = [amplitude; 0]*pi/180;
% undamped linearized natural period
T = 2*pi*sqrt(g/l);
%%
opts = optimoptions('fmincon','display','iter','MaxFunEvals',100000);

% number of points in optimization
N_opt = 101;
% initial guess for period time
T_init = 1*T;


Aineq = [];
bineq = [];
Aeq = [];
beq = [];
lb = [];
ub = [];


start = tic;
[T_EDMD, cost] = upper_level_pendulum_hard(L_0, L_u, C, x0, T_init, N_opt, opts);
tspan = linspace(0, T_EDMD, N_opt);
elapsed_time_fmincon = toc(start);
disp(['Elapsed time fmincon: ', num2str(elapsed_time_fmincon)])
dt = tspan(2);
[Z, U_EDMD, cost] = lower_level_pendulum_hard(L_0, L_u, C, x0, N_opt, T_EDMD);

X_EDMD = C*Z;
q_EDMD = X_EDMD(1, :);
qdot_EDMD = X_EDMD(2, :);


%% solve the original problem
tauspan = linspace(0, 1, N_opt);
dtau = tauspan(2);

% take the EDMD solution as the initial guess
initial_guess = [reshape(X_EDMD, numel(X_EDMD), []); U_EDMD'; T_EDMD];
[Z, opt_cost, ~, Output] = fmincon(@(Z) NLP_cost(Z, N_opt, n, dtau), initial_guess, Aineq, bineq, Aeq, beq, lb, ub, ...
    @(Z)NLP_constraint(Z, x0, dtau, @(t, x, u)pendulum_ode(t, x, u, param)), opts);

X_vec = Z(1:n*N_opt);
U_NLP = Z(n*N_opt+1:end-1);
T_NLP = Z(end);

X_NLP = reshape(X_vec, [n, N_opt]);
q_NLP = X_NLP(1, :);
qdot_NLP = X_NLP(2,:);
tspan_NLP = tauspan*T_NLP;
%% Plot results
fontsz = 16;
figure('Color','white')
subplot(3, 1, 1)
plot(tspan_NLP(1:end-1), U_NLP(1:end-1), LineWidth=2)
hold on
plot(tspan(1: end-1), U_EDMD(1:end-1), LineWidth=2, LineStyle="--");
grid on
ylabel('$u^\ast~\big[{}^\circ/\sqrt{l_\circ/g}\big]$', Interpreter='latex')
legend({'$\mathcal{P}$', '$\hat{\mathcal{P}}$'}, Interpreter="latex", FontSize=fontsz)


subplot(3, 1, 2)
plot(tspan_NLP, q_NLP(1,:)*180/pi, LineWidth=2)
hold on
plot(tspan, q_EDMD(1,:)*180/pi, LineWidth=2, LineStyle=":")
ylabel('$q^\ast~\big[{}^\circ/\sqrt{l_\circ/g}\big]$', Interpreter='latex')
title('$q$', Interpreter='latex', FontSize=fontsz)
grid on

subplot(3, 1, 3)
plot(tspan_NLP, qdot_NLP*180/pi, LineWidth=2)
hold on
plot(tspan, qdot_EDMD*180/pi, LineWidth=2, LineStyle=":")
xlabel('t [s]')
ylabel('$\dot{q}^\ast~\big[{}^\circ/\sqrt{l_\circ/g}\big]$', Interpreter='latex')
title('$\dot{q}$', Interpreter='latex', FontSize=fontsz)
grid on
