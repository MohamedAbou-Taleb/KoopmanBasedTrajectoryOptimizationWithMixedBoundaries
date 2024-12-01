function M = massMatrix(q, param)
a = param.a;
b = param.b;
m = param.m;
m_h = param.m_h;
l = param.l;

theta_stance = q(2);
theta_swing = q(1);

M = zeros(2);
M(1, 1) = m*b^2;
M(1, 2) = -m*l*b*cos(theta_stance-theta_swing);
M(2, 1) = M(1, 2);
M(2, 2) = (m_h+m)*l^2 + m*a^2;
end

