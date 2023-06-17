% Multirotor Dynamics class
%
% Should work for any simulator, vehicle, or operating system
%
% Based on
%
%     @inproceedings{DBLPconf/icra/BouabdallahMS04,
%       author    = {Samir Bouabdallah and Pierpaolo Murrieri and Roland Siegwart},
%       title     = {Design and Control of an Indoor Micro Quadrotor},
%       booktitle = {Proceedings of the 2004 {IEEE} International Conference on Robotics and
%                   Automation, {ICRA} 2004, April 26 - May 1, 2004, New Orleans, LA, {USA}},
%       pages     = {4393--4398},
%       year      = {2004},
%       crossref  = {DBLPconf/icra/2004},
%       url       = {https% doi.org/10.1109/ROBOT.2004.1302409},
%       doi       = {10.1109/ROBOT.2004.1302409},
%       timestamp = {Sun, 04 Jun 2017 010000 +0200},
%       biburl    = {https% dblp.org/rec/bib/conf/icra/BouabdallahMS04},
%       bibsource = {dblp computer science bibliography, https% dblp.org}
%     }
%
% Copyright (C) 2019 Simon D. Levy
%
% MIT License

classdef MultirotorDynamics
    % Abstract class for multirotor dynamics.
    
    properties(Constant, Access=public)
        
        % Position map for state vector
        STATE_X         = 1;
        STATE_X_DOT     = 2;
        STATE_Y         = 3;
        STATE_Y_DOT     = 4;
        STATE_Z         = 5;
        STATE_Z_DOT     = 6;
        STATE_PHI       = 7;
        STATE_PHI_DOT   = 8;
        STATE_THETA     = 9;
        STATE_THETA_DOT = 10;
        STATE_PSI       = 11;
        STATE_PSI_DOT   = 12;
        
        % universal constants
        g = 9.80665% might want to allow this to vary!
        
    end
    
    properties (Access=private)
        
        % A frame is defined by its mixer and parameters
        mixer;
        
        % We usuall start on ground, but can start in air for testing
        airborne;        
        
        % Initialize inertial frame acceleration in NED coordinates
        inertialAccel;
        
    end % private properties
    
    properties (Access=protected)
        
        % Frame constants
        params;

        % State vector and its first derivative
        x;
        dxdt;
        
        % Torque clockwise
        Omega;  
        
        % Values computed in Equation 6
        U1;     % total thrust
        U2;     % roll thrust right
        U3;     % pitch thrust forward
        U4;     % yaw thrust clockwise
                   
    end % protected properties
    
    methods (Access=public)
        
        function obj = setMotors(obj, motorvals)
            % Uses motor values to implement Equation 6.
            % motorvals in interval [0,1]
            
            % Convert the  motor values to radians per second
            omegas = motorvals * obj.params.maxrpm * pi / 30;
            
            % Compute overall torque from omegas before squaring
            obj.Omega = sum(obj.mixer(:,3)' .* omegas);
            
            % Overall thrust is sum of squared omegas
            omegas2 = omegas.^2;
            obj.U1 = obj.params.b * sum(omegas2);
            
            % Use the squared Omegas to implement the rest of Eqn. 6
            obj.U2 = obj.params.l * obj.params.b * sum(obj.mixer(:,1)' .* omegas2);
            obj.U3 = obj.params.l * obj.params.b * sum(obj.mixer(:,2)' .* omegas2);
            obj.U4 = obj.params.d * sum(obj.mixer(:,3)' .* omegas2);
            
        end
        
        function obj = update(obj, dt)
            % Updates state using  time in seconds since previous update
            
            % Use the current Euler angles to rotate the orthogonal thrust vector into the inertial frame.
            % Negate to use NED.
            euler = [obj.x(obj.STATE_PHI), obj.x(obj.STATE_THETA), obj.x(obj.STATE_PSI)];
            accelNED = transforms.bodyZToInertiall(-obj.U1 / obj.params.m, euler);
            
            % We're airborne once net downward acceleration goes below zero
            netz = accelNED(3) + obj.g;
            
            % If we're not airborne, we become airborne when downward acceleration has become negative
            if ~obj.airborne
                obj.airborne = netz < 0;
            end
            
            % Once airborne, we can update dynamics
            if obj.airborne

                % Update first temporal derivative of entire state in Equation 12 (x1, x3, x5, ...)
                obj.dxdt(obj.STATE_X:2:end) = obj.x(obj.STATE_X_DOT:2:end);
                
                % Compute second derivatives in Equation 12 (x2, x4, x6, ...)
                obj = obj.computeSecondDerivatives();
                
                % Compute state as first temporal integral of first temporal derivative
                obj.x = obj.x + dt * obj.dxdt;
                
                % Once airborne, inertial-frame acceleration is same as NED acceleration
                obj.inertialAccel = accelNED;
                
            end
            
        end % update()
        
        function s = getState(obj)
            % Returns a copy of the state vector as a tuple
            s = obj.x;
        end
        
        function obj = setPose(obj, pose, airborne)
            
            if nargin < 3
                airborne = false;
            end
            
            obj.x(obj.STATE_X:2:end) = pose;
            
            obj.airborne = airborne;
            
        end
        
        function c = motorCount(obj)
            c = length(obj.mixer);
        end
        
        function r = rollMixer(obj)
            r = obj.mixer(:,1)';
        end
        
    end
    
    methods (Access=protected)
        
        function obj = MultirotorDynamics(params, mixer, n)
            % Constructor
            % Initializes kinematic pose, with flag for whether we're airbone (helps with testing gravity).
            % airborne allows us to start on the ground (default) or in the air (e.g., gravity test)
            
            % Default to twelve-dimensional state vector
            if nargin < 3
                n = 12;
            end
            
            % Store constants
            obj.params = params;
            obj.mixer = mixer;
            
            % Always start at location (0,0,0) with zero velocities
            obj.x    = zeros(1, n);
            obj.dxdt = zeros(1, n);
            
            % Values computed in Equation 6
            obj.U1 = 0;     % total thrust
            obj.U2 = 0;     % roll thrust right
            obj.U3 = 0;     % pitch thrust forward
            obj.U4 = 0;     % yaw thrust clockwise
            obj.Omega = 0;  % torque clockwise
            
            % Initialize inertial frame acceleration in NED coordinates
            obj.inertialAccel = transforms.bodyZToInertiall(-obj.g, [0,0,0]);
            
            % We're not airborne yet
            obj.airborne = false;
            
        end
        
        function obj = computeSecondDerivatives(obj)
            % Implements Equation 12 computing temporal second derivative of state.
            % Override to implement more complicated dynamics.
            
            % Use the current Euler angles to rotate the orthogonal thrust vector into the inertial frame.
            % Negate to use NED.            
            euler = [obj.x(obj.STATE_PHI), obj.x(obj.STATE_THETA), obj.x(obj.STATE_PSI)];
            accelNED = transforms.bodyZToInertiall(-obj.U1 / obj.params.m, euler);         

            % Second temporal derivative of position
            obj.dxdt(obj.STATE_X_DOT) = accelNED(1);
            obj.dxdt(obj.STATE_Y_DOT) = accelNED(2);
            obj.dxdt(obj.STATE_Z_DOT) = accelNED(3) + obj.g;

            % Second temporal derivative of orientation
            obj = obj.computeOrientationSecondDerivatives();

        end
        
        function obj = computeOrientationSecondDerivatives(obj)
            
            % Shorthand
            p = obj.params;
            phidot = obj.x(obj.STATE_PHI_DOT);
            thedot = obj.x(obj.STATE_THETA_DOT);
            psidot = obj.x(obj.STATE_PSI_DOT);

            obj.dxdt(obj.STATE_PHI_DOT)   = psidot * thedot * (p.Iy - p.Iz) / p.Ix - p.Jr / p.Ix * thedot * obj.Omega + obj.U2 / p.Ix;
            obj.dxdt(obj.STATE_THETA_DOT) = -(psidot * phidot * (p.Iz - p.Ix) / p.Iy + p.Jr / p.Iy * phidot * obj.Omega + obj.U3 / p.Iy);
            obj.dxdt(obj.STATE_PSI_DOT)   = thedot * phidot * (p.Ix - p.Iy) / p.Iz + obj.U4 / p.Iz;        
        
        end
        
    end  % protected instance methods
    
end % classdef
