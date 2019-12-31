classdef parameters
  
  properties
    b;
    d;
    m;
    l;
    Ix;
    Iy;
    Iz;
    Jr;
    
    maxrpm;
    
  end
  
  methods
    
    function self = parameters(b, d, m, l, Ix, Iy, Iz, Jr, maxrpm)
      
      self.b = b;
      self.d = d;
      self.m = m;
      self.l = l;
      self.Ix = Ix;
      self.Iy = Iy;
      self.Iz = Iz;
      self.Jr = Jr;

      self.maxrpm = maxrpm;   
      
    end
  
  end
  
end
