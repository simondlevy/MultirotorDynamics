% Dynamics class for octo-X frames using ArduPilot motor layout
% 
%        5CCW   1CW
%
%    7CW           3CCW
%
%             ^
%
%    6CW           8CCW
%
%        2CCW   4CW
% 
% Copyright (C) 2019 Simon D. Levy
% 
% MIT License

classdef OctoXAPDynamics < MultirotorDynamics
  
    methods

        function obj = OctoXAPDynamics(params)
            obj = obj@MultirotorDynamics(params, 8);
        end

        function f = u2(~,  o)
            % roll right
            f =  (C1*o(2) + C1*o(5) + C2*o(6) + C2*o(7)) - (C1*o(1) + C2*o(3) + C1*o(4) + C2*o(8));
        end
       
        function f = u3(~,  o)
            % pitch forward
            f = (C2*o(2) + C2*o(4) + C1*o(6) + C1*o(8)) - (C2*o(1) + C1*o(3) + C2*o(5) + C1*o(7));
        end
       
        function f = u4(~,  o)
            % yaw cw
            f = (o(3) + o(4) + o(5) + o(6)) - (o(1) + o(2) + o(7) + o(8));
        end

        function d = motorDirection(i)
            %motor direction for animation
            dir = [+1, +1, -1, -1, -1, -1, +1, +1];
            d = dir(i);
        end

    end

end
