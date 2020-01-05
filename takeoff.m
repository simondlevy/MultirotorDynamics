% Run simple altitude-hold PID controller to test dynamics
%
% Usage:
%
%
%   takeoff(dur, dt) runs for DUR seconds with an update period of DT seconds
%
%   takeoff(dt, dur, file) also saves the results to a MAT file
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
    if nargin < 2
        fprintf('Usage: takeoff(duration, [dt]\n')
        return
    end

    % Create a wait-bar to display progress
    wb = waitbar(0);

    % Create PID controller
    pid  = AltitudePidController(ALTITUDE_TARGET, ALT_P, VEL_P, VEL_I, VEL_D);

    % Create dynamics
    dyn = DjiPhantomDynamics;

    % Initialize arrays for plotting
    tvals = linspace(0, dur, dur*1/dt);
    uvals = zeros(size(tvals));
    zvals = zeros(size(tvals));
    vvals = zeros(size(tvals));

    % Motors are initially off
    u = 0;

    % Loop for duration
    for k = 1:length(tvals)
        
        % Set all the motors to the value obtained from the PID controller
        dyn = dyn.setMotors(u*ones(1,4));

        % Update the dynamics
        dyn = dyn.update(dt);

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

        % Update the wait-bar
        t = tvals(k);
        waitbar(t/dur, wb, sprintf('%3.2f/%3.2f sec', t, dur))
        
        % Get correction from PID controller
        u = pid.u(z, v, dt);

        % Constrain correction to [0,1] to represent motor value
        u = max(0, min(1, u));

        % Track values
        zvals(k) = z;
        vvals(k) = v;
        uvals(k) = u;

    end

    % Clsoe the wait-bar
    close(wb)

    % Plot results
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
