% Function to display a quadcopter in 3D
%
% Copyright (C) 2019 Simon D. Levy
%
% MIT License

function showquad(phi, theta, psi, x, y, z)

plot3(x, y, z, 'or','MarkerSize',5,'MarkerFaceColor','r')
axis([-10 10 -10 10 0 10])
drawnow

end

