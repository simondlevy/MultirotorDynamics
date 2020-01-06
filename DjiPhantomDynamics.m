% Dynamics class for DJI Inspire
%
% Copyright (C) 2019 Simon D. Levy
%
% MIT License

classdef DjiPhantomDynamics < QuadXAPDynamics

    methods

      function obj = DjiPhantomDynamics
          
              % Estimated
              params.b = 5.E-06;  % force constatnt [F=b*w^2]
              params.d = 2.E-06;  % torque constant [T=d*w^2]
  
              % https:%www.dji.com/phantom-4/info
              params.m = 1.380;   % mass [kg]
              params.l = 0.350;   % arm length [m]
  
              % Estimated
              params.Ix = 2;      % [kg*m^2] 
              params.Iy = 2;      % [kg*m^2] 
              params.Iz = 3;      % [kg*m^2] 
              params.Jr = 38E-04; % prop inertial [kg*m^2] 

              params.maxrpm = 15000;  
                        
              obj = obj@QuadXAPDynamics(params); 
      end

    end

end
