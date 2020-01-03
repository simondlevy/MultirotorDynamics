%
% Test QuadDisplay class
%
% Copyright
%

N = 50;

qd = QuadDisplay;

for k =1:50
    
    qd.show(0, 0, 5, 0, 0, k*pi/N)
    
    pause(.01)
    
end
