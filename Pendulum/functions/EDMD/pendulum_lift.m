function Z = pendulum_lift(X)
% lifting function for gEDMD
x1 = X(1, :);
x2 = X(2, :);

Z = [X;
    sin(x1);
    cos(x1);
    x2.*sin(x1);
    x2.*cos(x1);
    x2.^2.*sin(x1);
    x2.^2.*cos(x1);
    x2.^3.*sin(x1);
    x2.^3.*cos(x1);
    x2.^2;
    x2.^3];
end

