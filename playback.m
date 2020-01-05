% Plays back kinematic data
%
% Usage:
%
%   playback(a) where A is an Nx7 matrix of kinematic data
% 
% Copyright (C) 2019 Simon D. Levy
%
% MIT License

function playback(a, aviname)

    % Set up a quadcopter display
    qd = QuadDisplay;

    % Start timing
    tic

    % Start at first frame
    k = 1;

    if nargin > 1
        vw = VideoWriter(aviname);
        open(vw);
    end

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
        qdshow(qd, k, a)

        % Write to video file if indicated
        if nargin > 1
            writeVideo(vw, getframe(gcf));
        end

    end

    % Display the final frame
    qdshow(qd, length(a), a)

    % Cleanup
    if nargin > 1
        close(vw);
    end

end

function qdshow(qd, k, a)
    qd.show(a(k, 2), a(k,3), -a(k,4), a(k,5), a(k,6), a(k,7), sprintf('t=%3.2f/%3.2f', a(k,1), a(end,1)))
end
