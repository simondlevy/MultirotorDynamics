% Abstract mixer class for multirotors
% 
% Copyright (C) 2020 Simon D. Levy
% 
% MIT License

classdef (Abstract) Mixer

    properties (Access=private)

        motorDirections; 

    end

    methods (Access=protected)

        function obj = Mixer(motorDirections)

            obj.motorDirections = motorDirections;

        end

    end

    methods (Access=public)
        
        function u = run(obj, demands)

            n = length(obj.motorDirections);

            u = zeros(1, n);

            for i = 1:n
                u(i) = sum(demands .* obj.motorDirections(i,:)); 
            end

        end

    end
   
end
