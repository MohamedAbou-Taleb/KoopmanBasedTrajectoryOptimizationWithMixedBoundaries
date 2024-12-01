function [stance_leg_handle, swing_leg_handle] = animate_CGW(q,  time, param)
a = param.a;
b = param.b;
l = param.l;
N = length(time);

theta_swing = q(1,1);
theta_stance = q(2,1);

x_hip = 0;
y_hip = l*cos(theta_stance);


stance_foot = [l*sin(theta_stance); y_hip-l*cos(theta_stance)];
swing_foot = [x_hip + l*sin(theta_swing);  y_hip - l*cos(theta_swing) ];
delta_x = -l*sin(theta_stance);
nframes = N; % # of frames to draw between [0, t] units of time
filename = 'CGW';


% leg visualization parameters
swing_leg_color = 0.3*[1,1,1]*0;
stance_leg_color = [0,0,0];
% stance_leg_color = [0.3,1,0.4];
leg_width = 10;

% create figure and handles
% figure('WindowState','maximized')
figure
% number of floor lines 
num_lines = 50;
xlim_array = [-1, 1];
[X_floor, Y_floor] = get_floor_coordinates(delta_x, num_lines, 2*(xlim_array(2)-xlim_array(1)));
floor_line_handle = plot(X_floor, Y_floor, Color = 'black', LineWidth=4);
hold on
plot([-2, 2], [0, 0], LineWidth=6, Color='black')
stance_leg_handle = plot([x_hip; stance_foot(1, 1)],[y_hip; stance_foot(2,1)], LineWidth=leg_width, Color=stance_leg_color);

circ_radius = 0.1;
circ_interv = linspace(0,2*pi, 40);
circ_coord = [circ_radius*cos(circ_interv); circ_radius*sin(circ_interv)];
swing_leg_handle = plot([x_hip; swing_foot(1, 1)], [y_hip; swing_foot(2, 1)], LineWidth=leg_width, Color=swing_leg_color);

main_body = patch(x_hip+circ_coord(1, :), y_hip+circ_coord(2, :), [0, 0, 1]);

xlim( xlim_array)
ylim([-0.2, 1.5])
pbaspect([xlim_array(2)-xlim_array(1), 1.6, 1])




% number of points used in animation
numPoints = 950;

step_size = ceil(N/numPoints)+1;
% step_size = 1;
new_time = linspace(0, time(end), numPoints);
delay_time = new_time(2);

ax = gca; % autoscales by default
% set(ax,'position',[0 0 1 1],'units','normalized')
set(gcf,'Position',[405 54.6000 703.2000 706.4000])
set(gca,'position',[-0.5 0 2 1])
% axis equal
grid off
set(ax,'XTickLabel',[]);
set(ax,'YTickLabel',[]);
ax.XAxis.Visible = 'off';
ax.YAxis.Visible = 'off';
nframes = length(1:step_size:N);
frames(nframes) = struct('cdata',[],'colormap',[]); 
frames(1) = getframe(ax);
k = 2;
for i = 2:step_size:N
    draw_CGW(q(:, i),  param, stance_leg_handle, swing_leg_handle,  main_body, ...
        floor_line_handle, circ_coord, num_lines, 2*(xlim_array(2)-xlim_array(1)), true)
    pause(0.05)
    drawnow limitrate
    frames(k) = getframe(ax);
    k = k+1;
end
stance_leg_handle.Color = swing_leg_color;
swing_leg_handle.Color = stance_leg_color;
drawnow
% for i = 2:step_size:N
%     draw_CGW(q(:, i),  param, stance_leg_handle, swing_leg_handle,  main_body, ...
%         floor_line_handle, circ_coord, num_lines, 2*(xlim_array(2)-xlim_array(1)), true)
%     pause(0.05)
%     drawnow limitrate
%     frames(k) = getframe(ax);
%     k = k+1;
% end
% for i = 2:step_size:N
%     draw_CGW([0,1;1,0]*q(:, i),  param, stance_leg_handle, swing_leg_handle,  main_body, ...
%         floor_line_handle, circ_coord, num_lines, 2*(xlim_array(2)-xlim_array(1)), false)
%     pause(0.05)
%     drawnow limitrate
%     frames(k) = getframe(ax);
%     k = k+1;
% end
if ~isempty(filename)
    % movie2gif(frames, [filename,'.gif'],'DelayTime',.05,'LoopCount',Inf)
    % movie2gif(frames, [filename,'.gif'],'DelayTime',time(2),'LoopCount',Inf)
    movie2gif(frames, [filename,'.gif'],'DelayTime',delay_time,'LoopCount',Inf)
end

end

