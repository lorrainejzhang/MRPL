classdef figure8ReferenceControl
    properties(Constant)
       v = 0.2;
       sf = 1;
       tf = obj.sf/obj.v;
       kth = 2*pi/obj.sf;
       kk = 15.1084;
    end
    
    properties
       ks; kv; tPause; Tf;
    end
    methods 
        function obj = figure8ReferenceControl(ks, kv, tPause)
            obj.ks = ks; obj.kv = kv; obj.tPause = tPause; 
            obj.Tf = (ks/kv) * obj.tf; 
        end

        function [V, w] = computeControl(obj, timeNow)
            if (timeNow < obj.tPause) || (timeNow > obj.Tf)
                V = 0; w = 0;
            else
                t = obj.kv/obj.ks * timeNow;
                s = obj.v * t;
                curv = (obj.kk/obj.ks) * sin(obj.kth*s);
                V = obj.kv*obj.v;
                w = curv*V;
            end
        end

        function duration = getTrajectoryDuration(obj)
            duration = 2*obj.tPause + obj.Tf;
        end
    end
end