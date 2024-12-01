function [XData, YData] = get_floor_coordinates(delta_x, num_lines, floor_length)
YData = zeros(2, num_lines);
YData(2,:) = -0.2; 
x_floor = linspace(-floor_length/2, floor_length/2, num_lines);
XData = [x_floor; x_floor + 0.15] - delta_x;
end

