% Plays back kinematic data
%
% Usage:
%
%   playback(a) where A is an Nx7 matrix of kinematic data
%
%   playback(a, aviname) also saves the movie to file AVINAME
% 
% Copyright (C) 2019 Simon D. Levy
%
% MIT License

function playback(a, aviname)

    % Set up a quadcopter display
    veh = VehicleDisplay;

    % Start timing
    tic

    % Start at first frame
    k = 1;

    % Open a video output file if indicated
    if nargin > 1
        vw = VideoWriter(aviname);
        open(vw);
    else
        vw = [];
    end

    % Start timing
    tic

    % Loop through the time values, interpolating as we go
    while true

        % Get the current actual time
        t = toc;

        % If current time exceeds total time, were' done
        if t > a(end,1) 
            break
        end

        % Find the frame closest to the current actual time
        while a(k,1) < t
            k = k+1;
        end

        % Display the current frame
        showvehicle(veh, k, a, vw)

    end

    % Display the final frame
    showvehicle(veh, length(a), a, vw)

    % Cleanup
    if ~isempty(vw)
        close(vw);
    end

end

function showvehicle(veh, k, a, vw)

    x     =  a(k,2);
    y     =  a(k,3);
    z     = -a(k,4); % NED => ENU
    phi   =  a(k,5);
    theta =  a(k,6);
    psi   =  a(k,7);
    veh.show(x, y, z, phi, theta, psi, ...
        sprintf('t=%3.2f/%3.2f  x=%+3.2f  y=%+3.2f  z=%3.2f', a(k,1), a(end,1), x, y, z))

    if ~isempty(vw)
        writeVideo(vw, getframe(gcf));
    end
end
