classdef trapReferenceControl < handle
    properties
        line;
        oldT;
        s; om;
        vmax; sf; 
        sgn
    end    
    methods
        function obj = trapReferenceControl(line, sf, sgn)
            obj.line = line;
            if (line)
                obj.vmax = .25;
            else
                obj.vmax = 2;
                sf = sf + pi/32; %small offset helps apparently
            end
            obj.oldT = 0;
            obj.s = 0; obj.om = 0;
            obj.sf = sf;
            obj.sgn = sgn;
        end
        function [V, w] = computeControl(obj, t)
            amax = 3 * obj.vmax;
            tramp = obj.vmax/amax; %1/3 meter
            tf = (obj.sf + (obj.vmax^2) / amax) / obj.vmax;
            if (t < 0 || t > tf)
                P = 0;
            elseif (t < tramp)
                P = amax * t;
            elseif (tf - t < tramp)
                P = amax * (tf - t);
            elseif (tramp < t && t < tf - tramp)
                P = obj.vmax;
            else
                P = 0;
            end
            P = obj.sgn * P;
            if (obj.line)
                V = P; w = 0;
                obj.s = obj.s + (V * (t - obj.oldT));
                obj.oldT = t;
            else
                V = 0; w = P;
                obj.om = obj.om + (w * (t - obj.oldT));
                obj.oldT = t;
            end            
        end
        
        function V = getVAtTime(obj, t)
            [V, ~] = obj.computeControl(t);
        end
        
        function w = getwAtTime(obj, t)
            [~, w] = obj.computeControl(t);
        end
        
        function pose = getPoseAtTime(obj, t)
            if (obj.line)
                x = obj.s; y = 0; th = 0;
            else
                x = 0; y = 0; th = obj.om;
            end
            pose  = [x ; y ; th];
        end
        
        function duration = getTrajectoryDuration(obj)
            duration = 4 + (1/3);
        end
        
        
    end
end