function x = RK4(xk, uk, dt, ode)
% Explicit fourth order fixed step Runge-Kutta integration
% xk: state at time k
% uk: input at time k
% dt: time step size
% ode: @(t, x, u) ode function handle; right hand side of the continuous-time ode 
s1 = ode(0, xk, uk);
s2 = ode(0, xk + dt/2*s1, uk);
s3 = ode(0, xk + dt/2*s2, uk);
s4 = ode(0, xk + dt*s3, uk);
x = xk + dt/6*(s1 + 2*s2 + 2*s3 + s4);
end

