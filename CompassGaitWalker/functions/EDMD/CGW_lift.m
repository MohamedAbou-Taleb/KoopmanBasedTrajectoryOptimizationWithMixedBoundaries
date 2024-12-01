function X_lift = CGW_lift(X)
% Lifting function for gEDMD
theta_swing = X(1,:);
theta_stance = X(2,:);
dtheta_swing = X(3,:);
dtheta_stance = X(4,:);

X_lift = [X; %1-4
          sin(theta_swing); %5
          cos(theta_swing); %6
          sin(theta_stance); %7
          cos(theta_stance); %8
          sin(theta_swing).^2; %9
          cos(theta_swing).^2; %10
          sin(theta_stance).^2; %11
          cos(theta_stance).^2; %12
          sin(theta_swing-theta_stance); %13
          cos(theta_swing-theta_stance); %14
          sin(theta_swing-theta_stance).^2; %15
          cos(theta_swing-theta_stance).^2; %16
          dtheta_swing.*sin(theta_swing-theta_stance); % 17
          dtheta_swing.*cos(theta_swing-theta_stance); % 18
          dtheta_stance.*sin(theta_swing-theta_stance); % 19
          dtheta_stance.*cos(theta_swing-theta_stance)]; % 20
sincos_q = [sin(theta_swing);
          cos(theta_swing);
          sin(theta_stance);
          cos(theta_stance)];
X_lift = [X_lift; 
          dtheta_swing.*sincos_q; % 21-24
          dtheta_stance.*sincos_q; % 25-28
          ones(1, size(X, 2))]; % 29
end

