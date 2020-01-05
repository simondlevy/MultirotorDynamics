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

    t = a(:,1);

    qd = QuadDisplay;

    tic

    while true

        if toc > t(end) 
            break
        end

        fprintf('%f\n', toc)

    end

end
