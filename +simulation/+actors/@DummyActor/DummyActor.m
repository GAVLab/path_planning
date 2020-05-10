classdef DummyActor < simulation.actors.AbstractActor
    %DUMMYACTOR This moron does not move.
    
    properties
        % Unique identifier
        id_ 
        
        % Dimensions
        safetyRadius_
        
        % Core state variables
        state_ % : Struct holding state variables
        %{
        position
        velocity
        latVel
        lonVel
        yawRate
        orientation
        steerAngle
        %}
        
        time_
        history_
    end
    
    methods
        function obj = DummyActor(initialState, id)
            %DUMMYACTOR Construct an instance of this class
            %   Detailed explanation goes here
            obj.state_ = initialState;
            obj.id_ = id;
            obj.safetyRadius_ = 1;
            
            obj.time_ = 0;
            obj.history_(:, 1) = [obj.time_; obj.state_.position(1); obj.state_.position(2)];
        end
        
        function obj = applyMotionModel(obj, dt)
            %APPLYMOTIONMODEL Update the state of the actor
            obj.time_ = obj.time_ + dt;
            obj.history_(:, end+1) = [obj.time_; obj.state_.position(1); obj.state_.position(2)];

        end
        
        function obj = calculateControlInput(obj, world, dt)
            %CALCULATECONTROLINPUT 
            obj.state_.steerAngle = 0;
        end
    end
end

