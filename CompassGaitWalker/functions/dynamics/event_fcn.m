function gap = event_fcn(x, param)
gamma = param.gamma;
q = x(1:2,:);
gap = q(1,:) + q(2,:) + 2*gamma;
end

