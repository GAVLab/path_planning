classdef FTGBicycle2Dof < simulation.actors.Bicycle2Dof 
    %FTGBICYCLE2DOF Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % [x; y] position to travel to
        goal_
        
        % Visbility limitations
        fov_ % : how far to each side we can see
        minRadius_ % : minimum turn radius
        
        isCollided;
        isGoalReached;

        filter
        
        timings_
    end
    
    methods
        function obj = FTGBicycle2Dof(initialState, id, goal)
            %FTGBICYCLE2DOF Construct an instance of this class
            %   Detailed explanation goes here
            import simulation.actors.FilterClass
            obj = obj@simulation.actors.Bicycle2Dof(initialState,id);
            obj.goal_ = goal; 
            obj.filter = simulation.actors.FilterClass();
            obj.fov_ = 60; % can see 90 deg to each side
            obj.minRadius_ = 5; % approx equal to wheelbase / tan(steer angle)
            obj.timings_ = [];
            
        end
        
        function obj = applyMotionModel(obj, dt)
            obj = applyMotionModel@simulation.actors.Bicycle2Dof(obj, dt);
            if norm(obj.state_.position - obj.goal_) < obj.safetyRadius_
                obj.isGoalReached = true;
            end
        end
        
        function obj = calculateControlInput(obj, world, dt)
            %CALCULATECONTROLINPUT
            
            % First do collision checking...
            for i = 1 : length(world.actors_)
                if isempty(world.actors_{i})
                    continue
                end
                if i == obj.id_
                    continue
                end
                obstacle = world.actors_{i};
                dist = norm(obstacle.state_.position - obj.state_.position);
                if dist < (2*obj.safetyRadius_)
                    obj.isCollided = true;
                    warning("Collision")
                    return;
                end
            end
            
            % Do filter stuff first
            %perform updates
            obj.filter.performUpdate(world,obj.id_,dt);
            
            [pos, covar] = obj.filter.predictForward(world,obj.id_);
            
            % Implement follow the gap-basic (Sezer and Gokasan, 2012)
            % TODO: Implement follow the gap standard
            % TODO: Implement noholonomic constraint
            
            % Tuning parameters
            alpha = 0.8; % weight coeff for gap
            beta = 1; % weight coeff for goal
            tic
            % Find angle to goal
            relativeGoalPosition = obj.goal_ - obj.state_.position;
            goalHeading = atan2d(relativeGoalPosition(2), relativeGoalPosition(1)) - rad2deg(obj.state_.orientation);
            if goalHeading > 180
                goalHeading = goalHeading - 360;
            elseif goalHeading < -180
                goalHeading = goalHeading + 360;
            end
            % Build gap array
            
            % gap_array(0) = -180
            % gap_array(end) = 180
            gap_array = ones(1, 361);
            gap_array(181 - obj.fov_ : 181 + obj.fov_) = 0;
            
            actors = obj.filter.remStates;
            
            dMin = inf;
            % Update these if nonholonomic constraints need to be invoked
            rightLimit = 1;
            leftLimit = length(gap_array);
            
            for i = 1 : length(actors)
                if isempty(actors{i})
                    continue;
                end
                if i == obj.id_
                    continue
                end
                
                xRob = obj.state_.position(1);
                yRob = obj.state_.position(2);
                
                xObs = actors{i}.xGlobal
                yObs = actors{i}.yGlobal
                
                radius = obj.safetyRadius_;
                
                % Check if in view
                relativePosition = [xObs - xRob; yObs - yRob]
                relativeHeading = atan2d(relativePosition(2), relativePosition(1)) - rad2deg(obj.state_.orientation)
                
                if abs(relativeHeading) > obj.fov_
                    continue
                end
                
                % This will get imaginary if safety radius is violated...
                dn = sqrt( (xObs - xRob)^2 + (yObs - yRob)^2 - (radius + radius)^2)
                d = sqrt( (xObs - xRob)^2 + (yObs - yRob)^2);
                
                if d < dMin
                    dMin = d;
                end
                
                if imag(dn) ~= 0
                    warning("You have collided");
                    obj.isCollided = true;
                end
                
                dn = real(dn);
                % How wide is the object as an angle?
                phiObs = acosd( dn / d );
                phiLeft = relativeHeading + phiObs;
                phiRight = relativeHeading - phiObs;
                
                if (phiLeft > obj.fov_)
                    phiLeft = obj.fov_;
                end
                if (phiRight < -obj.fov_)
                    phiRight = -obj.fov_;
                end
                
                leftBound = round(phiLeft + 180 + 1);
                rightBound = round(phiRight + 180 + 1);
                
                gap_array(rightBound : leftBound) = 1;
            end
            
            gap_array(1 : rightLimit) = 1;
            gap_array(leftLimit : end) = 1;
            
            % Find largest gap
            
            currStart = 1;
            currSize = 0;
            maxStart = 1;
            maxSize = 0;
            
            currIdx = 1;
            while currIdx <= length(gap_array)
                currStart = currIdx;
                currSize = 0;
                
                while (currIdx <= length(gap_array) && gap_array(currIdx) == 0)
                    % In a gap!
                    currSize = currSize + 1;
                    currIdx = currIdx + 1;
                end
                if currSize > maxSize
                    maxSize = currSize;
                    maxStart = currStart;
                end
                
                currIdx = currIdx + 1;
                
            end
            if currSize > maxSize
                maxSize = currSize;
                maxStart = currStart;
            end
            
            gapStart = maxStart;
            gapStartHeading = gapStart - obj.fov_ - 1;
            gapEnd = maxStart + maxSize - 1;
            gapEndHeading = gapEnd - obj.fov_ - 1;
            
            gapCenterHeading = mean([gapStartHeading, gapEndHeading]);
            
            % Calculate steer angle
            
            num = (alpha / dMin) * deg2rad(gapCenterHeading) + beta * deg2rad(goalHeading);
            den = (alpha / dMin) + beta;
            phiFinal = num / den;
            
            if phiFinal > obj.steerLimit_
                phiFinal = obj.steerLimit_;
            elseif phiFinal < -obj.steerLimit_
                phiFinal = - obj.steerLimit_;
            end
            obj.timings_(end+1) = toc;
            obj.state_.steerAngle = phiFinal;

            gap_array(round(goalHeading) + 180 + 1) = 2;
            
            %size(gap_array)
            gap_array(round(rad2deg(phiFinal)) + 180 + 1) = 3;
            rad2deg(phiFinal)
            size(gap_array)
            try
                plot([-180 : 1 : 180], gap_array(1:361));
                set(gca, 'XDir', 'reverse');
                pause(0.01)
            catch
                size(gap_array)
            end
        end
        
    end
end

