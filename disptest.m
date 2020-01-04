%
% Test QuadDisplay class
%
% Copyright
%

N = 50;

qd = QuadDisplay;

for k =1:50
    
    qd.show(0, 0, 5, k*pi/N, 0, 0)
    
    pause(.01)
    
end
