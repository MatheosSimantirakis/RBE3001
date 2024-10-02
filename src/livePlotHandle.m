classdef livePlotHandle < handle
    
    properties
        degPlotArray;
        posPlotArray;
        pathArray;
        degIndex;
        posIndex;
        pathIndex;
        startTime;
        havePause;
        havePath;
        haveBody;
    end
    
    methods
        function self = livePlotHandle()
           self.degPlotArray = zeros(5,0);
           self.posPlotArray = zeros(4,0);
           self.pathArray = zeros(3,0);
           self.degIndex = 1;
           self.posIndex = 1;
           self.pathIndex = 1;
           self.startTime = 0;
           self.havePause = true;
           self.haveBody = true;
           self.havePath = true;
        end
    end
end

