function Q = Q_plus(q, param)
a = param.a;
b = param.b;
m = param.m;
m_h = param.m_h;
l = param.l;
g = param.g;

theta_swing = q(1);
theta_stance = q(2);

alpha = 0.5*(theta_swing-theta_stance);

Q = zeros(2);
Q(1, 1) = m*b*(b-l*cos(2*alpha));
Q(1, 2) = m*l*(l-b*cos(2*alpha)) + m*a^2 + m_h*l^2;
Q(2, 1) = m*b^2;
Q(2, 2) = -m*b*l*cos(2*alpha);
end

