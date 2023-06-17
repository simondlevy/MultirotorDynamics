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
    
    properties (Constant, Access=private)
        mixer = [
            [-1,-1,-1];
            [+1,+1,-1];
            [+1,-1,+1]; 
            [-1,+1,+1]
            ];
    end
  
    methods

        function obj = QuadXAPDynamics(params)
            obj = obj@MultirotorDynamics(params, QuadXAPDynamics.mixer);
        end

    end

end
