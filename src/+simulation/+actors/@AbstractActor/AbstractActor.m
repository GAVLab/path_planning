classdef (Abstract) AbstractActor
    %ACTOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Abstract)
        % Unique identifier
        id_ 
        
        safetyRadius_
        
        % Core state variables
        state_ % : Struct holding state variables
        %{
        All classes should expose these in global coordinates:
        position
        velocity
        orientation
        latVel
        lonVel
        yawRate
        steerAngle
        
        Feel free to add any others you need for your model
        %}
        
        time_
        
        history_ % time, x, y
    end
    
    methods (Abstract)
        obj = applyMotionModel(obj, dt)
        
        obj = calculateControlInput(obj, world, dt)
    end
end

