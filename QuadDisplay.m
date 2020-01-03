% Function to display a quadcopter in 3D
%
% Copyright (C) 2019 Simon D. Levy
%
% MIT License

classdef QuadDisplay
    
    properties(Constant, Access=private)
        
        WORLD_SIZE           = 10;
        VEHICLE_SIZE         = 3;
        VEHICLE_COLOR        = 'r';
        VEHICLE_LINEWIDTH    = 2;
        PROPELLER_RADIUS     = 0.5;
        
    end
    
    properties(Access=private)
        d;
    end
    
    methods(Access=public)
        
        function obj = QuadDisplay
            s = obj.VEHICLE_SIZE / 2;
            obj.d = sqrt(s^2/2);
        end
        
        function show(obj, x, y, z, phi, theta, psi)
            
            obj.plotarm(x, y, z, -1, -1, +1, +1, phi, theta, psi)
            hold on
            obj.plotarm(x, y, z, -1, +1, +1, -1, phi, theta, psi)
            
            % Erase previous plot
            hold off
            
            axis([-obj.WORLD_SIZE obj.WORLD_SIZE -obj.WORLD_SIZE obj.WORLD_SIZE 0 obj.WORLD_SIZE])
            drawnow
        end
        
    end
    
    methods(Access=private)
        
        function plotarm(obj, x, y, z, dx1,dy1, dx2, dy2, phi, theta, psi)
            x1 = x+dx1*obj.d;
            x2 = x+dx2*obj.d;
            y1 = y+dy1*obj.d;
            y2 = y+dy2*obj.d;
            plot3([x1,x2], [y1,y2], [z,z], obj.VEHICLE_COLOR, 'LineWidth',obj.VEHICLE_LINEWIDTH)
            obj.plotprop(x1,y1,z)
            obj.plotprop(x2,y2, z)
        end
        
        function plotprop(obj, x, y, z)
            hold on
            th = 0:pi/50:2*pi;
            xunit = obj.PROPELLER_RADIUS * cos(th) + x;
            yunit = obj.PROPELLER_RADIUS * sin(th) + y;
            zunit = z * ones(size(xunit));
            plot3(xunit, yunit, zunit, obj.VEHICLE_COLOR, 'LineWidth',obj.VEHICLE_LINEWIDTH);
            hold off
        end
        
    end
    
end


