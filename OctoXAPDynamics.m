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
    
    properties(Access=private)
        C1;
        C2;
    end
  
    methods

        function obj = OctoXAPDynamics(params, C1, C2)
            obj = obj@MultirotorDynamics(params, 8);
            if nargin < 2
                C1 = 1;
                C2 = 1;
            end
            obj.C1 = C1;
            obj.C2 = C2;
        end

        function f = u2(obj,  o)
            % roll right
            f =  (obj.C1*o(2) + obj.C1*o(5) + obj.C2*o(6) + obj.C2*o(7)) - (obj.C1*o(1) + obj.C2*o(3) + obj.C1*o(4) + obj.C2*o(8));
        end
       
        function f = u3(obj,  o)
            % pitch forward
            f = (obj.C2*o(2) + obj.C2*o(4) + obj.C1*o(6) + obj.C1*o(8)) - (obj.C2*o(1) + obj.C1*o(3) + obj.C2*o(5) + obj.C1*o(7));
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
