% Dynamics class for DJI Inspire
%
% Copyright (C) 2019 Simon D. Levy
%
% MIT License

classdef dji_phantom_dynamics < quadxap_dynamics

    methods

      function self = dji_phantom_dynamics
        
          params = parameters(
  
              % Estimated
              5.E-06, % b
              2.E-06, % d
  
              % https:%www.dji.com/phantom-4/info
              1.380,  % m (kg)
              0.350,  % l (meters)
  
              % Estimated
              2,      % Ix
              2,      % Iy
              3,      % Iz
              38E-04, % Jr
              15000   % maxrpm
              );
              
              self = self@quadxap_dynamics(params);
  
      end

    end

end
