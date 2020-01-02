% Function to display a quadcopter in 3D
%
% Copyright (C) 2019 Simon D. Levy
%
% MIT License

function showquad(phi, theta, psi, x, y, z)

DISPLAY_SIZE = 10; 
VEHICLE_SIZE = 3;
VEHICLE_COLOR = 'r';

s = VEHICLE_SIZE / 2;
d = sqrt(s^2/2);

plot3([x-d,x+d], [y-d,y+d], [z,z], VEHICLE_COLOR)
hold on
plot3([x-d,x+d], [y+d,y-d], [z,z], VEHICLE_COLOR)
hold off

axis([-DISPLAY_SIZE DISPLAY_SIZE -DISPLAY_SIZE DISPLAY_SIZE 0 DISPLAY_SIZE])

drawnow

end

