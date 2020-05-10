classdef Bicycle2Dof < simulation.actors.AbstractActor
    %BICYCLE2DOF Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Unique identifier
        id_
        
        safetyRadius_
        
        steerLimit_
        
        % Core state variables
        state_ % : Struct holding state variables
        %{
        These are in global and are used by the world:
        position
        velocity
        orientation
        
        yawRate
        latVel : out the right side of the vehicle
        lonVel : out the front
        steerAngle : last calculated input
        
        %}
        
        time_
        history_
    end
    
    methods
        function obj = Bicycle2Dof(initialState, id)
            %BICYCLE2DOF Construct an instance of this class
            %   Detailed explanation goes here
            obj.state_ = initialState;
            obj.id_ = id;
            obj.safetyRadius_ = 1; 
            obj.steerLimit_ = 30 * pi/180; % rads
            obj.time_ = 0;
            obj.history_(:, 1) = [obj.time_; obj.state_.position(1); obj.state_.position(2);obj.state_.yawRate;obj.state_.latVel;obj.state_.orientation];
            
        end
        
        function obj = applyMotionModel(obj, dt)
            %APPLYMOTIONMODEL Update the state of the actor
            obj.time_ = obj.time_ + dt;
            
            input = obj.state_.steerAngle;
            
            if input > obj.steerLimit_
                input = obj.steerLimit_;
            elseif input < -obj.steerLimit_
                input = -obj.steerLimit_;
            end
            
            velMag = sqrt(obj.state_.latVel^2 + obj.state_.lonVel^2);
            
            % I'm not going to discretize but that wouldn't be a bad
            % idea...
            %xDot = A * [obj.state_.yawRate; obj.state_.latVel] + B * input;
            % yawAccel = xDot(1);
            % latAccel = xDot(2);
            
            [yawAccel, latAccel] = obj.bicycleModel(input, ...
                obj.state_.lonVel, obj.state_.latVel, velMag, ...
                obj.state_.yawRate);
            
            obj.state_.yawRate = obj.state_.yawRate + yawAccel * dt;
            obj.state_.latVel = obj.state_.latVel + latAccel * dt;
            obj.state_.orientation = ...
                obj.state_.orientation + obj.state_.yawRate * dt;
            
            obj.state_.velocity(1) = ...
                obj.state_.lonVel * cos(obj.state_.orientation) + ...
                obj.state_.latVel * sin(obj.state_.orientation);
            obj.state_.velocity(2) = ...
                obj.state_.lonVel * sin(obj.state_.orientation) - ...
                obj.state_.latVel * cos(obj.state_.orientation);
            
            obj.state_.position(1) = ...
                obj.state_.position(1) + dt * obj.state_.velocity(1);
            obj.state_.position(2) = ...
                obj.state_.position(2) + dt * obj.state_.velocity(2);
            
            obj.history_(:, end+1) = [obj.time_; obj.state_.position(1); obj.state_.position(2);obj.state_.yawRate;obj.state_.latVel;obj.state_.orientation];
        end
        
        function obj = calculateControlInput(obj, world, dt)
            %CALCULATECONTROLINPUT
            u = .5*sin(5 * obj.time_);
            obj.state_.steerAngle = u;
        end
        
        function [yawAccel , latAccel] = bicycleModel(obj, steerAngle,longVel,latVel,velMag,yawRate)
            %BICYCLEMODEL Summary of this function goes here
            %   Detailed explanation goes here
            
            %define vehicle parameters (g35 vehicle parameters)
            caf = 2*4.5837e+004; %corner stiffness front
            car = 2*7.6394e+004; %corner stiffness rear
            
            L = 2.8499;%length of wheel base
            a = .48*L;%distance from cg to front axle
            b =  .52*L;%distance from cg to rear axle
            
            Iz = 2400; %yaww inertia
            m = 1528.2; %vehicle mass
            
            %handling characteristics
            c0 = caf + car;
            c1 = a*caf - b*car;
            c2 = a^2*caf + b^2*car;
            
            %components of the A matrix
            a11 = -c2 / (Iz*longVel);
            a12 = -c1 / (Iz*longVel);
            a21 = (-c1 / (m*longVel)) - velMag;
            a22 = -c0 / (m*longVel);
            
            b11 = (a*caf / Iz);
            b22 = caf / m;
            
            %components of the B matrix
            
            states = [a11 , a12 ; a21 a22  ] * [yawRate ; latVel] + [b11 ; b22]*steerAngle;
            
            yawAccel = states(1);
            
            latAccel = states(2);
            
        end
        
        
    end
end

