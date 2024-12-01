function h = forces(q, dqdt, u, param)
a = param.a;
b = param.b;
m = param.m;
m_h = param.m_h;
l = param.l;
g = param.g;

theta_swing = q(1);
theta_stance = q(2);


C = zeros(2);
C(1, 2) = m*l*b*sin(theta_stance-theta_swing)*dqdt(2);
C(2, 1) = -m*l*b*sin(theta_stance-theta_swing)*dqdt(1);

G = [m*b*g*sin(theta_swing); -(m_h*l + m*a + m*l)*g*sin(theta_stance)];
B = [1; -1];
h = -C*dqdt - G + B*u;
end

