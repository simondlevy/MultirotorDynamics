% Simple PID controller 
% 
% Copyright (C) 2019 Simon D. Levy
% 
% MIT License

classdef PidController
    
    properties (Access=private)
        
        WINDUP_MAX_DEFAULT = 10;
        
        target;
        
        posP;
        velP;
        velI;
        velD;
        
        windupMax;
        posTarget;
        lastError;
        integralError;
        
    end
    
    methods (Access=public)
        
        function obj = PidController(target, posP, velP, velI, velD, windupMax)
            
            if nargin < 6
                windupMax = obj.WINDUP_MAX_DEFAULT;
            end
            
            obj.target = target;
            
            obj.posP = posP;
            obj.velP = velP;
            obj.velI = velI;
            obj.velD = velD;
            
            obj.windupMax = windupMax;
            
            obj.integralError = 0;
            
        end
        
        function c = u(obj, alt, vel, dt)
            
            % Compute v setpoint and error
            velTarget = (obj.target - alt) * obj.posP;
            velError = velTarget - vel;
            
            % Update error integral and error derivative
            obj.integralError =  obj.integralError + velError * dt;
            obj.integralError = obj.constrainAbs(obj.integralError + velError * dt, obj.windupMax);
            deltaError = 0;
            if abs(obj.lastError) > 0
                deltaError = (velError - obj.lastError) / dt;
            end
            obj.lastError = velError;
                        
            % Compute control u
            c = obj.velP * velError + obj.velD * deltaError + obj.velI * obj.integralError;
        end
        
    end
    
    methods(Static, Access=private)
        
        function c = constrainAbs(x, lim)
            c = min(max(-lim, x), +lim);
        end
        
    end
    
end % classdef
