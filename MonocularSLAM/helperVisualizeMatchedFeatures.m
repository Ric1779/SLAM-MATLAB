classdef helperVisualizeMatchedFeatures < handle
%helperVisualizeMatchedFeatures show the matched features in a frame
%
%   This is an example helper class that is subject to change or removal 
%   in future releases.

%   Copyright 2019-2020 The MathWorks, Inc.

    properties (Access = private)
        Image
        
        Feature

        hFig
    end
    
    methods (Access = public)
        
        function obj = helperVisualizeMatchedFeatures(I, featurePoints)
            locations= featurePoints.Location;
            
            % Plot image
            obj.hFig  = figure;
            hAxes = newplot(obj.hFig); 
            
            % Set figure visibility and position
            obj.hFig.Visible = 'on';
            movegui(obj.hFig, [300 220]);
            
            % Show the image
            obj.Image = imshow(I, 'Parent', hAxes, 'Border', 'tight');
            title(hAxes, 'Matched Features in Current Frame');
            hold(hAxes, 'on');
            
            % Plot features
            plot(featurePoints, hAxes, 'ShowOrientation',false, ...
                'ShowScale',false);
            obj.Feature = findobj(hAxes.Parent,'Type','Line'); 
        end 
        
        function updatePlot(obj, I, featurePoints, currFrameIdx)
            locations = featurePoints.Location;
            obj.Image.CData   = I;
            obj.Feature.XData = locations(:,1);
            obj.Feature.YData = locations(:,2);
            drawnow limitrate
            % Save the figure
            %file_name = ['./images/matchedFeatures/cameraPlot_' num2str(currFrameIdx) '.png'];
            %saveas(obj.hFig, file_name);
            
            % Display a message indicating that the figure has been saved
            %disp(['Figure saved as: ' file_name]);
        end
    end
end