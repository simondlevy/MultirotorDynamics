##Multirotor Dynamics class
##
##Should work for any simulator, vehicle, or operating system
## 
##Based on
## 
##    @inproceedings{DBLPconf/icra/BouabdallahMS04,
##      author    = {Samir Bouabdallah and Pierpaolo Murrieri and Roland Siegwart},
##      title     = {Design and Control of an Indoor Micro Quadrotor},
##      booktitle = {Proceedings of the 2004 {IEEE} International Conference on Robotics and
##                  Automation, {ICRA} 2004, April 26 - May 1, 2004, New Orleans, LA, {USA}},
##      pages     = {4393--4398},
##      year      = {2004},
##      crossref  = {DBLPconf/icra/2004},
##      url       = {https#doi.org/10.1109/ROBOT.2004.1302409},
##      doi       = {10.1109/ROBOT.2004.1302409},
##      timestamp = {Sun, 04 Jun 2017 010000 +0200},
##      biburl    = {https#dblp.org/rec/bib/conf/icra/BouabdallahMS04},
##      bibsource = {dblp computer science bibliography, https#dblp.org}
##    }
## 
##Copyright (C) 2019 Simon D. Levy
## 
##MIT License

classdef (Abstract) multirotor_dynamics
% Abstract class for multirotor dynamics.  

  properties

    % Position map for state vector
    _STATE_X         = 1;
    _STATE_X_DOT     = 2;
    _STATE_Y         = 3;
    _STATE_Y_DOT     = 4;
    _STATE_Z         = 5;
    _STATE_Z_DOT     = 6;
    _STATE_PHI       = 7;
    _STATE_PHI_DOT   = 8;
    _STATE_THETA     = 9;
    _STATE_THETA_DOT = 10;
    _STATE_PSI       = 12;
    _STATE_PSI_DOT   = 12;
    
    % universal constants
    g = 9.80665% might want to allow this to vary!

    % instance variables
    _p;
    _motorCount;

    _omegas;
    _omegas2;

    % Always start at location (0,0,0) with zero velocities
    _x;
    _dxdt;

    _airborne;

    % Values computed in Equation 6
    _U1;     % total thrust
    _U2;     % roll thrust right
    _U3;     % pitch thrust forward
    _U4;     % yaw thrust clockwise
    _Omega;  % torque clockwise

    % Initialize inertial frame acceleration in NED coordinates
    _inertialAccel;

    % We usuall start on ground, but can start in air for testing
    _airborne;
    
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
     
      self._p = params;
      self._motorCount = motorCount;

      self._omegas  = zeros(1, motorCount);
      self._omegas2 = zeros(1, motorCount);

      # Always start at location (0,0,0) with zero velocities
      self._x    = zeros(1, 12);
      self._dxdt = zeros(1, 12);

      self._airborne = false;

      # Values computed in Equation 6
      self._U1 = 0;     % total thrust
      self._U2 = 0;     % roll thrust right
      self._U3 = 0;     % pitch thrust forward
      self._U4 = 0;     % yaw thrust clockwise
      self._Omega = 0;  % torque clockwise

      # Initialize inertial frame acceleration in NED coordinates
      self._inertialAccel = self._bodyZToInertiall(-self.g, [0,0,0]);

      # We usuall start on ground, but can start in air for testing
      self._airborne = airborne;
        
    end
    
    function f = _bodyZToInertiall(self, bodyZ, rotation)
      % _bodyToInertial method optimized for body X=Y=0

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

     function setMotors(self, motorvals)
        % Uses motor values to implement Equation 6.
        % motorvals in interval [0,1]

        % Convert the  motor values to radians per second
        self._omegas = self._computeMotorSpeed(motorvals); % rad/s

        % Compute overall torque from omegas before squaring
        self._Omega = self.u4(self._omegas);

        % Overall thrust is sum of squared omegas
        %self._omegas2 = self._omegas**2
        %self._U1 = sum(self._p.b * self._omegas2)

        % Use the squared Omegas to implement the rest of Eqn. 6
        %self._U2 = self._p.l * self._p.b * self.u2(self._omegas2)
        %self._U3 = self._p.l * self._p.b * self.u3(self._omegas2)
        %self._U4 = self._p.d * self.u4(self._omegas2)
     end

     function s = _computeMotorSpeed(self, motorvals)
        % Computes motor speed (rad/s) based on motor value in [0,1]
        s = motorvals * self._p.maxrpm * pi / 30;
     end

  end

end


#        
##    def update(self, dt)
##        '''
##        Updates state.
##        dt time in seconds since previous update
##        '''
##    
##        # Use the current Euler angles to rotate the orthogonal thrust vector into the inertial frame.
##        # Negate to use NED.
##        euler = ( self._x[6], self._x[8], self._x[10] )
##        accelNED = MultirotorDynamics._bodyZToInertiall(-self._U1 / self._p.m, euler)
##
##        # We're airborne once net downward acceleration goes below zero
##        netz = accelNED[2] + MultirotorDynamics.g
##
##        # If we're not airborne, we become airborne when downward acceleration has become negative
##        if not self._airborne
##            self._airborne = netz < 0
##
##        # Once airborne, we can update dynamics
##        if self._airborne
##
##            # Compute the state derivatives using Equation 12
##            self._computeStateDerivative(accelNED, netz)
##
##            # Compute state as first temporal integral of first temporal derivative
##            self._x += dt * self._dxdt
##
##            # Once airborne, inertial-frame acceleration is same as NED acceleration
##            self._inertialAccel = accelNED.copy()
##
##    def getState(self)
##        '''
##        Returns a copy of the state vector as a tuple
##        '''
##        return tuple(self._x)
##
##    def _computeStateDerivative(self, accelNED, netz)
##        '''
##        Implements Equation 12 computing temporal first derivative of state.
##        Should fill _dxdx[0..11] with appropriate values.
##        accelNED acceleration in NED inertial frame
##        netz accelNED[2] with gravitational constant added in
##        phidot rotational acceleration in roll axis
##        thedot rotational acceleration in pitch axis
##        psidot rotational acceleration in yaw axis
##        '''
##        p = self._p
##        x = self._x
##        Omega = self._Omega
##        U2 = self._U2
##        U3 = self._U3
##        U4 = self._U4
## 
##        phidot = x[MultirotorDynamics._STATE_PHI_DOT]
##        thedot = x[MultirotorDynamics._STATE_THETA_DOT]
##        psidot = x[MultirotorDynamics._STATE_PSI_DOT]
##
##        self._dxdt[0]  = x[MultirotorDynamics._STATE_X_DOT]                                                    # x'
##        self._dxdt[1]  = accelNED[0]                                                                          # x''
##        self._dxdt[2]  = x[MultirotorDynamics._STATE_Y_DOT]                                                    # y'
##        self._dxdt[3]  = accelNED[1]                                                                          # y''
##        self._dxdt[4]  = x[MultirotorDynamics._STATE_Z_DOT]                                                    # z'
##        self._dxdt[5]  = netz                                                                                 # z''
##        self._dxdt[6]  = phidot                                                                               # phi'
##        self._dxdt[7]  = psidot * thedot * (p.Iy - p.Iz) / p.Ix - p.Jr / p.Ix * thedot * Omega + U2 / p.Ix    # phi''
##        self._dxdt[8]  = thedot                                                                               # theta'
##        self._dxdt[9]  = -(psidot * phidot * (p.Iz - p.Ix) / p.Iy + p.Jr / p.Iy * phidot * Omega + U3 / p.Iy) # theta''
##        self._dxdt[10] = psidot                                                                               # psi'
##        self._dxdt[11] = thedot * phidot * (p.Ix - p.Iy) / p.Iz + U4 / p.Iz                                   # psi''
##
#
#
##    def _inertialToBody(inertial, rotation)
##    
##        phi, theta, psi = rotation
##
##        cph = cos(phi)
##        sph = sin(phi)
##        cth = cos(theta)
##        sth = sin(theta)
##        cps = cos(psi)
##        sps = sin(psi)
##
##        R = [[cps * cth,                    cth * sps,                         -sth],
##             [cps * sph * sth - cph * sps,  cph * cps + sph * sps * sth,  cth * sph],
##             [sph * sps + cph * cps * sth,  cph * sps * sth - cps * sph,  cph * cth]]
##
##        return dot(R, inertial)
##
##    def _bodyToInertial(body, rotation, inertial)
##        '''
##         Frame-of-reference conversion routines.
##         
##         See Section 5 of http#www.chrobotics.com/library/understanding-euler-angles
##        '''
##
##        phi, theta, psi = rotation
##
##        cph = cos(phi)
##        sph = sin(phi)
##        cth = cos(theta)
##        sth = sin(theta)
##        cps = cos(psi)
##        sps = sin(psi)
##
##        R = [[cps * cth,  cps * sph * sth - cph * sps,  sph * sps + cph * cps * sth],
##             [cth * sps,  cph * cps + sph * sps * sth,  cph * sps * sth - cps * sph],
##             [-sth,       cth * sph,                                      cph * cth]]
##
##        return dot(R, body)
##
##    def _eulerToQuaternion(eulerAngles)
##
##        # Convenient renaming
##        phi, the, psi = eulerAngles / 2
##
##        # Pre-computation
##        cph = cos(phi)
##        cth = cos(the)
##        cps = cos(psi)
##        sph = sin(phi)
##        sth = sin(the)
##        sps = sin(psi)
##
##        # Conversion
##        return [[ cph * cth * cps + sph * sth * sps],
##                [cph * sth * sps - sph * cth * cps],
##                [-cph * sth * cps - sph * cth * sps],
##                [cph * cth * sps - sph * sth * cps]]
##

