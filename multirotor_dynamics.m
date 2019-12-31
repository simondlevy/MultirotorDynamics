%Multirotor Dynamics class
%
%Should work for any simulator, vehicle, or operating system
% 
%Based on
% 
%    @inproceedings{DBLPconf/icra/BouabdallahMS04,
%      author    = {Samir Bouabdallah and Pierpaolo Murrieri and Roland Siegwart},
%      title     = {Design and Control of an Indoor Micro Quadrotor},
%      booktitle = {Proceedings of the 2004 {IEEE} International Conference on Robotics and
%                  Automation, {ICRA} 2004, April 26 - May 1, 2004, New Orleans, LA, {USA}},
%      pages     = {4393--4398},
%      year      = {2004},
%      crossref  = {DBLPconf/icra/2004},
%      url       = {https%doi.org/10.1109/ROBOT.2004.1302409},
%      doi       = {10.1109/ROBOT.2004.1302409},
%      timestamp = {Sun, 04 Jun 2017 010000 +0200},
%      biburl    = {https%dblp.org/rec/bib/conf/icra/BouabdallahMS04},
%      bibsource = {dblp computer science bibliography, https%dblp.org}
%    }
% 
%Copyright (C) 2019 Simon D. Levy
% 
%MIT License

classdef (Abstract) multirotor_dynamics
% Abstract class for multirotor dynamics.  

  properties

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
    STATE_PSI       = 12;
    STATE_PSI_DOT   = 12;
    
    % universal constants
    g = 9.80665% might want to allow this to vary!

    % instance variables
    p;
    motorCount;

    omegas;
    omegas2;

    % Always start at location (0,0,0) with zero velocities
    x;
    dxdt;

    % Values computed in Equation 6
    U1;     % total thrust
    U2;     % roll thrust right
    U3;     % pitch thrust forward
    U4;     % yaw thrust clockwise
    Omega;  % torque clockwise

    % Initialize inertial frame acceleration in NED coordinates
    inertialAccel;

    % We usuall start on ground, but can start in air for testing
    airborne;
    
  end

  methods (Abstract)

    % roll right
    u2(self, omega2)

    % pitch forward
    u3(self, omega2)

    % yaw cw
    u4(self, omega2)

  end

  methods
    
    function self = multirotor_dynamics(params, motorCount, airborne)
      % Constructor
      % Initializes kinematic pose, with flag for whether we're airbone (helps with testing gravity).
      % airborne allows us to start on the ground (default) or in the air (e.g., gravity test)
        
      if nargin < 3
        airborne = true;
      end
     
      self.p = params;
      self.motorCount = motorCount;

      self.omegas  = zeros(1, motorCount);
      self.omegas2 = zeros(1, motorCount);

      % Always start at location (0,0,0) with zero velocities
      self.x    = zeros(1, 12);
      self.dxdt = zeros(1, 12);

      self.airborne = false;

      % Values computed in Equation 6
      self.U1 = 0;     % total thrust
      self.U2 = 0;     % roll thrust right
      self.U3 = 0;     % pitch thrust forward
      self.U4 = 0;     % yaw thrust clockwise
      self.Omega = 0;  % torque clockwise

      % Initialize inertial frame acceleration in NED coordinates
      self.inertialAccel = self.bodyZToInertiall(-self.g, [0,0,0]);

      % We usuall start on ground, but can start in air for testing
      self.airborne = airborne;
        
    end
    
     function self = setMotors(self, motorvals)
        % Uses motor values to implement Equation 6.
        % motorvals in interval [0,1]

        % Convert the  motor values to radians per second
        self.omegas = self.computeMotorSpeed(motorvals); % rad/s

        % Compute overall torque from omegas before squaring
        self.Omega = self.u4(self.omegas);

        % Overall thrust is sum of squared omegas
        %self.omegas2 = self.omegas**2
        %self.U1 = sum(self.p.b * self.omegas2)

        % Use the squared Omegas to implement the rest of Eqn. 6
        %self.U2 = self.p.l * self.p.b * self.u2(self.omegas2)
        %self.U3 = self.p.l * self.p.b * self.u3(self.omegas2)
        %self.U4 = self.p.d * self.u4(self.omegas2)
     end

     function s = computeMotorSpeed(self, motorvals)
        % Computes motor speed (rad/s) based on motor value in [0,1]
        s = motorvals * self.p.maxrpm * pi / 30;
     end

  end % methods
  
  methods(Static)
      
    function f = bodyZToInertiall(bodyZ, rotation)
      % bodyToInertial method optimized for body X=Y=0

      phi   = rotation(1);
      theta = rotation(2);
      psi   = rotation(3);

      cph = cos(phi);
      sph = sin(phi);
      cth = cos(theta);
      sth = sin(theta);
      cps = cos(psi);
      sps = sin(psi);

      % This is the rightmost column of the body-to-inertial rotation matrix
      R = [sph * sps + cph * cps * sth, cph * sps * sth - cps * sph, cph * cth];

      f = bodyZ * R;

    end
     
  end % methods(Static)

end % classdef


%        
%    def update(self, dt)
%        '''
%        Updates state.
%        dt time in seconds since previous update
%        '''
%    
%        % Use the current Euler angles to rotate the orthogonal thrust vector into the inertial frame.
%        % Negate to use NED.
%        euler = ( self.x[6], self.x[8], self.x[10] )
%        accelNED = MultirotorDynamics._bodyZToInertiall(-self.U1 / self.p.m, euler)
%
%        % We're airborne once net downward acceleration goes below zero
%        netz = accelNED[2] + MultirotorDynamics.g
%
%        % If we're not airborne, we become airborne when downward acceleration has become negative
%        if not self.airborne
%            self.airborne = netz < 0
%
%        % Once airborne, we can update dynamics
%        if self.airborne
%
%            % Compute the state derivatives using Equation 12
%            self.computeStateDerivative(accelNED, netz)
%
%            % Compute state as first temporal integral of first temporal derivative
%            self.x += dt * self.dxdt
%
%            % Once airborne, inertial-frame acceleration is same as NED acceleration
%            self.inertialAccel = accelNED.copy()
%
%    def getState(self)
%        '''
%        Returns a copy of the state vector as a tuple
%        '''
%        return tuple(self.x)
%
%    def _computeStateDerivative(self, accelNED, netz)
%        '''
%        Implements Equation 12 computing temporal first derivative of state.
%        Should fill _dxdx[0..11] with appropriate values.
%        accelNED acceleration in NED inertial frame
%        netz accelNED[2] with gravitational constant added in
%        phidot rotational acceleration in roll axis
%        thedot rotational acceleration in pitch axis
%        psidot rotational acceleration in yaw axis
%        '''
%        p = self.p
%        x = self.x
%        Omega = self.Omega
%        U2 = self.U2
%        U3 = self.U3
%        U4 = self.U4
% 
%        phidot = x[MultirotorDynamics.STATE_PHI_DOT]
%        thedot = x[MultirotorDynamics.STATE_THETA_DOT]
%        psidot = x[MultirotorDynamics.STATE_PSI_DOT]
%
%        self.dxdt[0]  = x[MultirotorDynamics.STATE_X_DOT]                                                    % x'
%        self.dxdt[1]  = accelNED[0]                                                                          % x''
%        self.dxdt[2]  = x[MultirotorDynamics.STATE_Y_DOT]                                                    % y'
%        self.dxdt[3]  = accelNED[1]                                                                          % y''
%        self.dxdt[4]  = x[MultirotorDynamics.STATE_Z_DOT]                                                    % z'
%        self.dxdt[5]  = netz                                                                                 % z''
%        self.dxdt[6]  = phidot                                                                               % phi'
%        self.dxdt[7]  = psidot * thedot * (p.Iy - p.Iz) / p.Ix - p.Jr / p.Ix * thedot * Omega + U2 / p.Ix    % phi''
%        self.dxdt[8]  = thedot                                                                               % theta'
%        self.dxdt[9]  = -(psidot * phidot * (p.Iz - p.Ix) / p.Iy + p.Jr / p.Iy * phidot * Omega + U3 / p.Iy) % theta''
%        self.dxdt[10] = psidot                                                                               % psi'
%        self.dxdt[11] = thedot * phidot * (p.Ix - p.Iy) / p.Iz + U4 / p.Iz                                   % psi''
%
%
%
%    def _inertialToBody(inertial, rotation)
%    
%        phi, theta, psi = rotation
%
%        cph = cos(phi)
%        sph = sin(phi)
%        cth = cos(theta)
%        sth = sin(theta)
%        cps = cos(psi)
%        sps = sin(psi)
%
%        R = [[cps * cth,                    cth * sps,                         -sth],
%             [cps * sph * sth - cph * sps,  cph * cps + sph * sps * sth,  cth * sph],
%             [sph * sps + cph * cps * sth,  cph * sps * sth - cps * sph,  cph * cth]]
%
%        return dot(R, inertial)
%
%    def _bodyToInertial(body, rotation, inertial)
%        '''
%         Frame-of-reference conversion routines.
%         
%         See Section 5 of http%www.chrobotics.com/library/understanding-euler-angles
%        '''
%
%        phi, theta, psi = rotation
%
%        cph = cos(phi)
%        sph = sin(phi)
%        cth = cos(theta)
%        sth = sin(theta)
%        cps = cos(psi)
%        sps = sin(psi)
%
%        R = [[cps * cth,  cps * sph * sth - cph * sps,  sph * sps + cph * cps * sth],
%             [cth * sps,  cph * cps + sph * sps * sth,  cph * sps * sth - cps * sph],
%             [-sth,       cth * sph,                                      cph * cth]]
%
%        return dot(R, body)
%
%    def _eulerToQuaternion(eulerAngles)
%
%        % Convenient renaming
%        phi, the, psi = eulerAngles / 2
%
%        % Pre-computation
%        cph = cos(phi)
%        cth = cos(the)
%        cps = cos(psi)
%        sph = sin(phi)
%        sth = sin(the)
%        sps = sin(psi)
%
%        % Conversion
%        return [[ cph * cth * cps + sph * sth * sps],
%                [cph * sth * sps - sph * cth * cps],
%                [-cph * sth * cps - sph * cth * sps],
%                [cph * cth * sps - sph * sth * cps]]
%

