function xdot = ode_fcn(t, x, u, param)
q = x(1:2);
dqdt = x(3:4);

M = massMatrix(q, param);
h = forces(q, dqdt, u, param);
xdot = zeros(4, 1);
xdot(1:2) = dqdt;
xdot(3:4) = M\h;
end

