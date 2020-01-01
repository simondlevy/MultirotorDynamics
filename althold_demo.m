%Run simple altitude-hold PID controller to test dynamics
%Copyright (C) 2019 Simon D. Levy
%MIT License


DURATION        = 30; % seconds
ALTITUDE_TARGET = 10; % meters

% PID params
ALT_P = 1.0;
VEL_P = 1.0;
VEL_I = 0;
VEL_D = 0;

% Time constant
DT = 0.001;

% Create PID controller
pid  = altitude_pid_controller(ALTITUDE_TARGET, ALT_P, VEL_P, VEL_I, VEL_D);

dyn = dji_phantom_dynamics();

% Initialize arrays for plotting
n = fix(DURATION/DT);
tvals = linspace(0, DURATION, n);
uvals = zeros(1,n);
zvals = zeros(1,n);
vvals = zeros(1,n);

% Motors are initially off
u = 0;

% Loop over time values
for k = 1:length(tvals)
    
    % Set all the motors to the value obtained from the PID controller
    dyn.setMotors(u*ones(1,4));
    
    % Update the dynamics
    dyn.update(.001);
    
    % Get the current vehicle state
    s = dyn.getState();
    
    % Extract altitude, vertical velocity from state, negating to handle NED coordinate system
    z = -s(4);
    v = -s(5);
    
    % Get correction from PID controller
    u = pid.u(z, v, DT);
    
    % Constrain correction to [0,1] to represent motor value
    u = max(0, min(1, u));
    
    % Track values
    k = k(0);
    uvals(k) = u;
    zvals(k) = z;
    vvals(k) = v;
    
end

% Plot results
do_subplot(tvals, zvals, 1, 'Altitude (m)')
do_subplot(tvals, vvals, 2, 'Velocity (m/s)')
do_subplot(tvals, uvals, 3, 'Motors')

function do_subplot(t, x, k, label)
subplot(3,1,k)
plot(t, x)
ylabel(label)
end

