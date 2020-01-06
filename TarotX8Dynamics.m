% Dynamics class for Tarot X8
%
% (Approximated for now)
%
% Copyright (C) 2020 Simon D. Levy
%
% MIT License

classdef TarotX8Dynamics < OctoXAPDynamics

    methods

      function obj = TarotX8Dynamics
          
              params.b  = 3.E-05;       % force constatnt [F=b*w^2]
              params.d  = 7.E-07;       % torque constant [T=d*w^2]
              params.m  = 4.2;          % mass [kg]
              params.l  = 0.375;        % arm length [m]
              params.Ix = 0.15;         % [kg*m^2]
              params.Iy = 0.15;         % [kg*m^2]
              params.Iz = 0.20;         % [kg*m^2]
              params.Jr = 2.765168e-05; % prop inertia [kg*m^2]

              params.maxrpm = 6875;  
                        
              obj = obj@OctoXAPDynamics(params); 
      end

    end

end
