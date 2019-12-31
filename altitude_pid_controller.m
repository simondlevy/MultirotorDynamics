classdef altitude_pid_controller
    
    properties
        
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
    
    methods
        
        function self = altitude_pid_controller(target, posP, velI, velD, windupMax)
            
            if nargin < 5
                windupMax = WINDUP_MAX_DEFAULT;
                endif
                
                self.target = target;
                self.posP = posP;
                self.velI = velI;
                self.velD = velD;
                self.windupMax = windupMax;
                
            end
        end
        
        function c = u(self, alt, vel, dt)
            
            % Compute v setpoint and error
            velTarget = (self.target - alt) * self.posP;
            velError = velTarget - vel;
            
            % Update error integral and error derivative
            self.integralError =  self.integralError + velError * dt;
            self.integralError = constrainAbs(self.integralError + velError * dt, self.windupMax);
            deltaError = 0;
            if abs(self.lastError) > 0
                deltaError = (velError - self.lastError) / dt;
            end
            self.lastError = velError;
            
            % Compute control u
            c = self.velP * velError + self.velD * deltaError + self.velI * self.integralError;
        end
        
    end
    
    methods(Static)
        
        function c = constrainAbs(x, lim)
            c = min(max(-lim, x), +lim);
        end
        
    end
    
end % classdef





