% Function to display a quadcopter in 3D
%
% Copyright (C) 2019 Simon D. Levy
%
% MIT License

classdef QuadDisplay

    properties(Constant, Access = private)

        WORLD_SIZE    = 10; 
        VEHICLE_SIZE  = 3;
        VEHICLE_COLOR = 'r';

    end

    properties(Access = private)
        d;
    end

    methods(Access = public)

        function obj = QuadDisplay
            s = obj.VEHICLE_SIZE / 2;
            obj.d = sqrt(s^2/2);
        end

        function show(obj, x, y, z, phi, theta, psi)
            plot3([x-obj.d,x+obj.d], [y-obj.d,y+obj.d], [z,z], obj.VEHICLE_COLOR)
            hold on
            plot3([x-obj.d,x+obj.d], [y+obj.d,y-obj.d], [z,z], obj.VEHICLE_COLOR)
            hold off
            axis([-obj.WORLD_SIZE obj.WORLD_SIZE -obj.WORLD_SIZE obj.WORLD_SIZE 0 obj.WORLD_SIZE])
            drawnow
        end

    end

end 


