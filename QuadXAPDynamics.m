% Dynamics class for quad-X frames using ArduPilot motor layout
% 
%     3cw   1ccw
%        \ /
%         ^
%        / \
%     2ccw  4cw
% 
% Copyright (C) 2019 Simon D. Levy
% 
% MIT License

classdef QuadXAPDynamics < MultirotorDynamics
  
    methods

        function obj = QuadXAPDynamics(params)
            obj = obj@MultirotorDynamics(params, 4);
        end

        function f = u2(~,  o)
            % roll right
            f = (o(2) + o(3)) - (o(1) + o(2));
        end
       
        function f = u3(~,  o)
            % pitch forward
            f = (o(2) + o(4)) - (o(1) + o(3));
        end
       
        function f = u4(~,  o)
            % yaw cw
            f = sum(o);
        end

        function d = motorDirection(i)
            %motor direction for animation
            dir = [-1, -1, +1, +1];
            d = dir(i);
        end

    end

end
