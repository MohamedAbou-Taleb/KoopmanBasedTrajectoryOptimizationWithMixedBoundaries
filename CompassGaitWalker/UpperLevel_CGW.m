% Koopman Based Trajectory Optimization Example: Compass-Gait Walker
clear all
close all
addpath(genpath([pwd, '/../utilities']))
addpath(genpath(pwd))
%% initialize parameters

g = 1;
m_h = 1/2;
m = 1/4;
a = 0.5;
b = 0.5;
l = a+b;
param_vec = [a, b, m, m_h, l, g];

gamma = 0;

param.l = l;
param.g = g;
param.m_h = m_h;
param.m = m;
param.a = a;
param.b = b;
param.gamma = gamma;

%% TO params
gamma = 0;
param.gamma = 0;

x0 = [-0.050588475300906;
    0.050588475300906;
    -0.089078478554002;
    -0.085316243206319];

T_passive = 1.954108488643065;
v_avg = 0.050746181547040;


%% sampling parameters
% number of samples
N = 45000;
% sampling bounds
theta_swing_max = 15*pi/180;
theta_stance_max = 15*pi/180;
dtheta_swing_max = 0.15;
dtheta_stance_max = 0.1;
sampling_bounds.theta_swing_max = theta_swing_max;
sampling_bounds.theta_stance_max = theta_stance_max;
sampling_bounds.dtheta_swing_max = dtheta_swing_max;
sampling_bounds.dtheta_stance_max = dtheta_stance_max;

%% apply eDMD
% regression parameters
lambda = 0;
threshold = 1e-7; %1e-7


[L_0, L_u, C] = get_bilinear_model_CGW(N, sampling_bounds, param_vec);
nz = length(L_0);

C_EDMD = eye(4, nz);
nu = 1;
n = 4;
%% run trajectory optimization
opts = optimset('TolX', 1e-7);
% number of discretization points
N_opt = 51;
% operating point / desired average speed
v_avg_opt = 0.05;

% Initial guess for x0 and T_init
T_init = 1*T_passive;
x0_T_init = [-0.0562
    0.0562
   -0.0844
   -0.0902
   2.2478];

Aineq = [];
bineq = [];
Aeq = [];
beq = [];
lb = [];
ub = [];

start = tic;
[x0_opt, T_opt, cost] = upper_level_CGW(L_0, L_u, C_EDMD, v_avg_opt, x0_T_init, N_opt, param, opts);
tspan = linspace(0, T_opt, N_opt);
elapsed_time_fmincon = toc(start);
disp(['Elapsed time fmincon: ', num2str(elapsed_time_fmincon)])
dt = tspan(2);
[X_U_T, U_EDMD, cost] = lower_level_CGW(L_0, L_u, C_EDMD, x0_opt, N_opt, T_opt, param);


X_EDMD = C_EDMD*X_U_T;
q_EDMD = X_EDMD(1:2, :);
qdot_EDMD = X_EDMD(3:4, :);

%% solve NLP 
tauspan = linspace(0, 1, N_opt);
dtau = tauspan(2);
optcon = optimoptions('fmincon','display','iter','MaxFunEvals',inf, 'MaxIterations', inf);
initial_guess = [reshape(X_EDMD, numel(X_EDMD), []); U_EDMD'; T_opt];
[X_U_T, opt_cost, ~, Output] = fmincon(@(X_U_T) cost_function_NLP(X_U_T, N_opt, n, dtau), initial_guess, [], [], [], [], [], [], ...
    @(X_U_T)nonlinear_constraints_NLP(X_U_T, v_avg_opt, dtau, @(t, x, u)ode_fcn(t, x, u, param), param), optcon);

X_vec = X_U_T(1:n*N_opt);
X_NLP = reshape(X_vec, n, N_opt);
q_NLP = X_NLP(1:2, :);
qdot_NLP = X_NLP(3:4,:);
U_NLP = X_U_T(n*N_opt+1:end-1);
T_NLP = X_U_T(end);
tspan_NLP = linspace(0, T_NLP, N_opt);

%%
animate_CGW(q_NLP, tspan_NLP, param);

%%
fontsz = 16;
figure
subplot(2,2,1)
plot(tspan_NLP, q_NLP(1,:)*180/pi, LineWidth=1.5)
hold on
plot(tspan, q_EDMD(1,:)*180/pi, LineWidth=1.5, LineStyle="--")

grid on
set(gca, 'XTickLabel', [])
ylabel('$\theta^\ast_{\mathrm{swing}}\big[{}^\circ/\sqrt{l_\circ/g}\big]$', Interpreter='latex', FontSize=fontsz)

subplot(2,2,2)
plot(tspan_NLP, q_NLP(2,:)*180/pi, LineWidth=1.5)
hold on
plot(tspan, q_EDMD(2,:)*180/pi, LineWidth=1.5, LineStyle="-.")

ylabel('$\theta^\ast_{\mathrm{stance}}\big[{}^\circ/\sqrt{l_\circ/g}\big]$', Interpreter='latex', FontSize=fontsz)
set(gca, 'XTickLabel', [])
legend({'$\mathcal{P}$', '$\hat{\mathcal{P}}$'}, Interpreter="latex", FontSize=fontsz)
grid on

subplot(2,2,3)
plot(tspan_NLP, qdot_NLP(1,:)*180/pi, LineWidth=1.5)
hold on
plot(tspan, X_EDMD(3,:)*180/pi, LineWidth=1.5, LineStyle="-.")


xlabel('$t\, [\sqrt{l_\circ/g}]$', Interpreter='latex', FontSize=fontsz)
ylabel('$\dot{\theta}^\ast_{\mathrm{swing}}\big[{}^\circ/\sqrt{l_\circ/g}\big]$', Interpreter='latex', FontSize=fontsz)
grid on

subplot(2,2,4)
plot(tspan_NLP, qdot_NLP(2,:)*180/pi, LineWidth=1.5)
hold on
plot(tspan, X_EDMD(4,:)*180/pi, LineWidth=1.5, LineStyle="-.")
ylabel('$\dot{\theta}^\ast_{\mathrm{stance}}\big[{}^\circ/\sqrt{l_\circ/g}\big]$', Interpreter='latex', FontSize=fontsz)
grid on
xlabel('$t\, [\sqrt{l_\circ/g}]$', Interpreter='latex', FontSize=fontsz)
%%
figure
% subplot(3, 1, 3)
stairs(tspan(1:end-1), U_EDMD(1:end-1), LineWidth=1.5)
hold on
stairs(tspan_NLP(1:end-1), U_NLP(1:end-1), LineWidth=1.5)

grid on
xlabel('$t\, [\sqrt{l_\circ/g}]$', Interpreter='latex', FontSize=fontsz)
ylabel('$u^\ast\, [m gl_\circ]$', Interpreter='latex', FontSize=fontsz)
legend({'$\mathcal{P}$', '$\hat{\mathcal{P}}$'}, Interpreter="latex", FontSize=fontsz)
