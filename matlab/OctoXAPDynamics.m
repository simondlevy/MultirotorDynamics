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

    properties (Constant, Access=private)
        mixer = [
                [-1, -1, +1 ]; % 1
                [+1, +1, +1 ]; % 2
                [-1, -1, -1 ]; % 3
                [-1, +1, -1 ]; % 4
                [+1, -1, -1 ]; % 5
                [+1, +1, -1 ]; % 6
                [+1, -1, +1 ]; % 7
                [-1, +1, +1 ]; % 8
            ];
    end

    properties(Access=private)
        C1;
        C2;
    end
  
    methods

        function obj = OctoXAPDynamics(params, C1, C2)
            obj = obj@MultirotorDynamics(params, OctoXAPDynamics.mixer);
            if nargin < 2
                C1 = 1;
                C2 = 1;
            end
            obj.C1 = C1;
            obj.C2 = C2;
        end

    end

end
