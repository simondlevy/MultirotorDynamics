% Run simple altitude-hold PID controller to test dynamics
%
% Usage:
%
%
%   takeoff(dur, dt) runs for DUR seconds with an update period of DT seconds
%
%   takeoff(dt, dur, csvlog) also saves the results to file CSVLOG
%
%   takeoff(dur) displays in real time
%
% Copyright (C) 2019 Simon D. Levy
%
% MIT License

function takeoff(dur, dt, csvlog)

    % Simulation params
    ALTITUDE_TARGET = 10;

    % PID params
    ALT_P = 1.0;
    VEL_P = 1.0;
    VEL_I = 0;
    VEL_D = 0;

    % Time constant
    if nargin < 1
        fprintf('Usage: takeoff(duration, [dt], [csvlog]\n')
        return
    elseif nargin < 2
        dt = 0;
        q = QuadDisplay;
    else
        f = waitbar(0);
    end

    % Create PID controller
    pid  = AltitudePidController(ALTITUDE_TARGET, ALT_P, VEL_P, VEL_I, VEL_D);

    % Create dynamics
    dyn = DjiPhantomDynamics;

    % Initialize arrays for plotting
    tvals = [];
    uvals = [];
    zvals = [];
    vvals = [];

    % Motors are initially off
    u = 0;

    % Start timing
    t = 0;
    tprev = 0;
    tic

    % Loop for duration
    while t < dur
        
        % Set all the motors to the value obtained from the PID controller
        dyn = dyn.setMotors(u*ones(1,4));

        % Update the dynamics
        dyn = dyn.update(.001);

        % Get the current vehicle state vector
        s = dyn.getState();

        % Extract values from state vector, negating to handle NED coordinate system
        phi   =  s(MultirotorDynamics.STATE_PHI);
        theta =  s(MultirotorDynamics.STATE_THETA);
        psi   =  s(MultirotorDynamics.STATE_PSI);
        x     =  s(MultirotorDynamics.STATE_X);
        y     =  s(MultirotorDynamics.STATE_Y);
        z     = -s(MultirotorDynamics.STATE_Z);
        v     = -s(MultirotorDynamics.STATE_Z_DOT);

        msg = sprintf('%3.2f/%3.2f sec', t, dur);

        if nargin > 1
            t = t + dt;
            waitbar(t/dur, f, msg)
        else

            % Update the timer
            t = toc;
            dt = t - tprev;
            tprev = t;

            % Show the vehicle
            q.show(x, y, z, phi, theta, psi, msg)

        end
        
        % Get correction from PID controller
        if dt > 0
            u = pid.u(z, v, dt);
        end

        % Constrain correction to [0,1] to represent motor value
        u = max(0, min(1, u));

        % Track values
        tvals = [tvals, t];
        zvals = [zvals, z];
        vvals = [vvals, v];
        uvals = [uvals, u];

    end

    if nargin > 1
        close(f)
    end

    % Plot results
    %figure
    make_subplot(tvals, zvals, 1, 'Altitude (m)', [0 ALTITUDE_TARGET+1])
    make_subplot(tvals, vvals, 2, 'Velocity (m/s)')
    make_subplot(tvals, uvals, 3, 'Motors', [-.1,1.1])

end

function make_subplot(t, x, k, label, ylims)
    subplot(3,1,k)
    plot(t, x)
    ylabel(label)
    if nargin > 4
        ylim(ylims)
    end
end
