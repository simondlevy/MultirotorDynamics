% Plays back kinematic data
%
% Usage:
%
%   playback(a) where A is an Nx7 matrix of kinematic data
% 
% Copyright (C) 2019 Simon D. Levy
%
% MIT License

function playback(a)

    % Get values from matrix
    tvals     = a(:,1);
    xvals     = a(:,2);
    yvals     = a(:,3);
    zvals     = a(:,4);
    phivals   = a(:,5);
    thetavals = a(:,6);
    psivals   = a(:,7);

    % Set up a quadcopter display
    qd = QuadDisplay;

    % Start timing
    tic

    % Start at first frame
    k = 1;

    % Loop through the time values, interpolating as we go
    while true

        % Get the current actual time
        t = toc;

        % If current time exceeds total time, were' done
        if toc > tvals(end) 
            break
        end

        % Find the frame closest to the current actual time
        while tvals(k) < t
            k = k+1;
        end

        % Display the quadcopter, negating Z for NED => ENU
        qd.show(xvals(k), yvals(k), -zvals(k), phivals(k), thetavals(k), psivals(k))

    end

end
