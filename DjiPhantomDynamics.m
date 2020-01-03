% Dynamics class for DJI Inspire
%
% Copyright (C) 2019 Simon D. Levy
%
% MIT License

classdef DjiPhantomDynamics < QuadXAPDynamics

    methods

      function obj = DjiPhantomDynamics
          
              % Estimated
              params.b = 5.E-06;
              params.d = 2.E-06;
  
              % https:%www.dji.com/phantom-4/info
              params.m = 1.380; % kg
              params.l = 0.350; % m
  
              % Estimated
              params.Ix = 2;      
              params.Iy = 2;      
              params.Iz = 3;      
              params.Jr = 38E-04; 
              params.maxrpm = 15000;  
                        
              obj = obj@QuadXAPDynamics(params); 
      end

    end

end