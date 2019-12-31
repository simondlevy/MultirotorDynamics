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

classdef quadxap_dynamics < multirotor_dynamics
  
    methods

        function self = quadxap_dynamics(params)
            self = self@multirotor_dynamics(params, 4);
        end

        function f = u2(self,  o)
            % roll right
            f = (o(1) + o(2)) - (o(0) + o(3));
        end
       
        function u3(self,  o)
            % pitch forward
            f = (o(1) + o(3)) - (o(0) + o(2));
        end
       
        function u4(self,  o)
            % yaw cw
            f = (o(0) + o(1)) - (o(2) + o(3));
        end

        function d = motorDirection(i)
            %motor direction for animation
            dir = [-1, -1, +1, +1];
            d = dir(i);
        end

    end

end
