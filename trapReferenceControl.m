classdef trapReferenceControl
    methods
        function [V, w] = computeControl(obj, t)
            vmax = 0.25;
            amax = 3 * 0.25;
            tramp = vmax/amax; %1/3 meter
            sf = 1;
            w = 0;
            tf = (sf + (vmax^2) / amax) / vmax;
            if (t < 0 || t > tf)
                V = 0;
            elseif (t < tramp)
                V = amax * t;
            elseif (tf - t < tramp)
                V = amax * (tf - t);
            elseif (tramp < t && t < tf - tramp)
                V = vmax;
            else
                V = 0;
            end
        end
        function duration = getTrajectoryDuration(obj)
            duration = 4 + (1/3);
        end
    end
end