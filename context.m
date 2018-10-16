classdef context < handle
    properties
        ti;
        robot;
        goodT;
        maxV = .2;
        feedbackOn;
    end
    methods
        function obj = context()
            obj.ti = tic();
            obj.robot = raspbot();
        end
        
        function T = getLocalTime(obj)
            T = toc(obj.ti);
        end
    end
end