% Geometric transforms library
%
% Copyright (C) 2019 Simon D. Levy
%
% MIT License

classdef transforms
    
    methods(Static, Access=public)
        
        %  Frame-of-reference conversion routines
        %  See Section 5 of httpwww.chrobotics.com/library/understanding-euler-angles
        
        function inertial = bodyZToInertiall(bodyZ, rotation)
            % bodyToInertial method optimized for body X=Y=0
            
            [cph, sph, cth, sth, cps, sps] = transforms.sincos(rotation);
            
            % This is the rightmost column of the body-to-inertial rotation matrix
            R = [sph * sps + cph * cps * sth; cph * sps * sth - cps * sph; cph * cth];
            
            inertial = (R * bodyZ')';
            
        end
        
        function body = inertialToBody(inertial, rotation)
            
            [cph, sph, cth, sth, cps, sps] = transforms.sincos(rotation);
            
            R = [cps * cth,                    cth * sps,                         -sth;
                cps * sph * sth - cph * sps,  cph * cps + sph * sps * sth,  cth * sph;
                sph * sps + cph * cps * sth,  cph * sps * sth - cps * sph,  cph * cth];
            
            body =  (R * inertial')';
            
        end
        
        function inertial = bodyToInertial(body, rotation)
            
            [cph, sph, cth, sth, cps, sps] = transforms.sincos(rotation);
            
            
            R = [cps * cth, cps * sph * sth - cph * sps,  sph * sps + cph * cps * sth;
                cth * sps,  cph * cps + sph * sps * sth,  cph * sps * sth - cps * sph;
                -sth,       cth * sph,                                      cph * cth];
            
            inertial = (R * body')';
            
        end
        
        function quat = eulerToQuaternion(euler)
            
            [cph, sph, cth, sth, cps, sps] = transforms.sincos(euler/2);
            
            quat =  [cph * cth * cps + sph * sth * sps, ...
                cph * sth * sps - sph * cth * cps, ...
                -cph * sth * cps - sph * cth * sps, ...
                cph * cth * sps - sph * sth * cps];
        end
        
        function r = rotation(phi, theta, psi)
            r = [];
        end
        
    end % public static methods
    
    methods (Static, Access=private)
        function [cph, sph, cth, sth, cps, sps] = sincos(angles)
            % pre-compute helper for angular transform matrices
            
            phi   = angles(1);
            theta = angles(2);
            psi   = angles(3);
            
            cph = cos(phi);
            sph = sin(phi);
            cth = cos(theta);
            sth = sin(theta);
            cps = cos(psi);
            sps = sin(psi);
            
        end
        
    end % private static methods
    
end % classdef



