function Q = Q_minus(q, param)
a = param.a;
b = param.b;
m = param.m;
m_h = param.m_h;
l = param.l;
g = param.g;

theta_swing = q(1);
theta_stance = q(2);

% alpha = 0.5*(theta_swing-theta_stance);
alpha = -0.5*(theta_swing-theta_stance);

Q = zeros(2);
Q(1, 1) = -m*a*b;
Q(1, 2) = -m*a*b + (m_h*l^2 + 2*m*a*l)*cos(2*alpha);
Q(2, 2) = -m*a*b;
end

