function draw_CGW(q, param, stance_leg_handle, swing_leg_handle, main_body_handle,...
    floor_line_handle,  circle_coord, num_lines, floor_length, first_step)
a = param.a;
b = param.b;
l = param.l;
% 
theta_swing = q(1);
theta_stance = q(2);



x_hip = 0;
y_hip = l*cos(theta_stance);

if first_step
    delta_x = -l*sin(theta_stance);
else
    delta_x = l*sin(theta_stance);
end
[X_floor, Y_floor] = get_floor_coordinates(delta_x, num_lines, floor_length);

stance_foot = [l*sin(theta_stance); y_hip - l*cos(theta_stance)];
swing_foot = [l*sin(theta_swing);  y_hip - l*cos(theta_swing)];

set(stance_leg_handle, 'XData', [x_hip; stance_foot(1)])
set(stance_leg_handle, 'YData', [y_hip; stance_foot(2)])

set(swing_leg_handle, 'XData', [x_hip; swing_foot(1)])
set(swing_leg_handle, 'YData', [y_hip; swing_foot(2)])

set(main_body_handle, 'XData', x_hip + circle_coord(1, :))
set(main_body_handle, 'YData', y_hip + circle_coord(2, :))

set(floor_line_handle, {'xdata'}, mat2cell(X_floor,  size(X_floor, 1),  ones(1, size(X_floor, 2))).')
set(floor_line_handle, {'ydata'}, mat2cell(Y_floor,  size(Y_floor, 1),  ones(1, size(Y_floor, 2))).')

drawnow limitrate
end

