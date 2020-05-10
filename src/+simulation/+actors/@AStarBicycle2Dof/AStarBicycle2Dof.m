classdef AStarBicycle2Dof < simulation.actors.Bicycle2Dof
    %ASTARBICYCLE2DOF Summary of this class goes here
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
        lookaheadDistance_
    end
    
    methods
        function obj = AStarBicycle2Dof(initialState, id, goal)
            %FTGBICYCLE2DOF Construct an instance of this class
            %   Detailed explanation goes here
            import simulation.actors.FilterClass
            obj = obj@simulation.actors.Bicycle2Dof(initialState,id);
            obj.goal_ = goal;
            obj.filter = simulation.actors.FilterClass();
            obj.timings_ = [];
            obj.lookaheadDistance_ = 5;
        end
        
        function obj = applyMotionModel(obj, dt)
            obj = applyMotionModel@simulation.actors.Bicycle2Dof(obj, dt);
            if norm(obj.state_.position - obj.goal_) < obj.safetyRadius_
                obj.isGoalReached = true;
            end
        end
        
        function path = reconstructPath(obj, currentNode, parentSet)
            %{
            total_path := {current}
            while current in cameFrom.Keys:
                current := cameFrom[current]
                total_path.prepend(current)
            return total_path
            %}
            totalPath = {currentNode};
            while ~isempty(parentSet{currentNode.id})
                currentNode = parentSet{currentNode.id};
                totalPath = [{currentNode} totalPath];
            end
            path = totalPath;
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
                    warning("Collision!")
                    return;
                end
            end
            
            % Do filter stuff first
            % perform updates
            obj.filter.performUpdate(world,obj.id_,dt)
            
            [pos, covar] = obj.filter.predictForward(world,obj.id_);
            tic;
            [path, map] = obj.calculatePathAStar(world, pos, covar);
            if isempty(path)
                return
            end
            obj.timings_(end+1) = toc;
            
            for i = 1 : numel(path)
               node = path{i};
               map(node.row, node.col) = 1;
            end
            
            
            
            
            % Pure Pursuit Control
            
            if numel(path) >= obj.lookaheadDistance_
                targetNode = path{obj.lookaheadDistance_};
            else
                targetNode = path{end};
            end
            targetRow = targetNode.row;
            targetCol = targetNode.col;
            [targetX, targetY] = world.idxToCoords([targetRow, targetCol]);
            
            %[row, col] = world.coordsToIdx(obj.state_.position);
            
            relPosition = [targetX - obj.state_.position(1); targetY - obj.state_.position(2)]

            goalHeading = atan2d(relPosition(2), relPosition(1)) - rad2deg(obj.state_.orientation);
            if goalHeading > 180
                goalHeading = goalHeading - 360;
            elseif goalHeading < -180
                goalHeading = goalHeading + 360;
            end
            
            phiFinal = deg2rad(goalHeading)
            if phiFinal > obj.steerLimit_
                phiFinal = obj.steerLimit_;
            elseif phiFinal < -obj.steerLimit_
                phiFinal = - obj.steerLimit_;
            end
            
            obj.state_.steerAngle = phiFinal;
        end
        
        function [path, map] = calculatePathAStar(obj, world, pos, covar)
            %% A* Trajectory Planning
            
            %% Build initial map
            
            %{
            Map info:
            obstacle = -1
            target = 0
            actor = 1
            space = 2
            openSet = 3
            closedSet = 4
            %}
            map = 2 * ones(size(world.map_));
            
            [actorRow, actorCol] = world.coordsToIdx(obj.state_.position);
            map(actorRow, actorCol) = 1;
            [goalRow, goalCol] = world.coordsToIdx(obj.goal_);
            map(goalRow, goalCol) = 0;
            
            %% Mask off current estimated positions
            actors = obj.filter.remStates;
            for i = 1 : length(world.actors_)
                if isempty(world.actors_{i})
                    continue;
                end
                if i == obj.id_
                    continue
                end
                
                xGlobal = actors{i}.xGlobal;
                yGlobal = actors{i}.yGlobal;
                
                [row, col] = world.coordsToIdx([xGlobal, yGlobal]);
                for k = -1*obj.safetyRadius_*world.gridResolution_ : 1 : 1*obj.safetyRadius_*world.gridResolution_
                    for j = -1*obj.safetyRadius_*world.gridResolution_ : 1 : 1*obj.safetyRadius_*world.gridResolution_
                        newRow = row + k;
                        newCol = col + j;
                        
                        if (newRow > 0 && newRow < size(map, 1) && newCol > 0 && newCol < size(map,2))
                            map(row + k, col + j) = -1;
                        end
                    end
                end
                map(row, col) = -1;
            end
            
            %% Mask off propagated estimates (skip first row, corresponds to self)
            for i = 2 : size(pos, 1)
                xPos = pos(i, 2);
                yPos = pos(i, 3);
                xPosCovar = covar(i, 2);
                yPosCovar = covar(i, 3);
                
                var = max([xPosCovar, yPosCovar]);
                boxDim = round(obj.safetyRadius_ + 3 * sqrt(var)) * world.gridResolution_;
                
                [row, col] = world.coordsToIdx([xPos; yPos]);
                
                for k = -boxDim : 1 : boxDim
                    for j = -boxDim : 1 : boxDim
                        newRow = row + k;
                        newCol = col + j;
                        
                        if (newRow > 0 && newRow < size(map, 1) && newCol > 0 && newCol < size(map,2))
                            map(row + k, col + j) = -1;
                        end
                    end
                end
            end
            
            
            %% Build needed data
            startNode.row = actorRow;
            startNode.col = actorCol;
            startNode.id = sub2ind(size(map), startNode.row, startNode.col);
            initGoalDist = sqrt((goalRow - startNode.row)^2 + (goalCol - startNode.col)^2);
            
            % Set of discovered nodes that need to be explored
            openSet = {startNode};
            
            % Set of nodes we can't explore.
            closedSet = {};
            for i = 1 : size(map, 1)
                for j = 1 : size(map, 2)
                    if map(i, j) == -1
                        obstacleNode.row = i;
                        obstacleNode.col = j;
                        obstacleNode.id = sub2ind(size(map), i, j);
                        closedSet{end+1} = obstacleNode;
                    end
                end
            end
            
            % parentSet(id) is the node immediately preceding node id
            parentSet = cell(numel(map), 1);
            
            gScore = inf * ones(numel(map), 1); % cost start -> node
            gScore(startNode.id) = 0;
            hScore = zeros(numel(map), 1); % cost node -> goal (est)
            hScore(startNode.id) = initGoalDist;
            fScore = inf * ones(numel(map), 1); % h + g
            fScore(startNode.id) = gScore(startNode.id) + hScore(startNode.id);
            
            %% Run algorithm!
            while ~isempty(openSet)
                % Try the node with the lowest score next
                idx = obj.findMinF(openSet, fScore);
                currNode = openSet{idx};
                map(currNode.row, currNode.col) = 4;
                
                if (currNode.row == goalRow && currNode.col == goalCol)
                    % A path to goal has the lowest score!
                    path = obj.reconstructPath(currNode, parentSet);
                    return
                end
                
                openSet(idx) = [];
                
                id = sub2ind(size(map), currNode.row, currNode.col);
                
                % Try each neighbor of current node
                neighbors = obj.getNeighbors(currNode, closedSet, map);
                for i = 1 : numel(neighbors)
                    neighbor = neighbors{i};
                    neighborId = sub2ind(size(map), neighbor.row, neighbor.col);
                    neighborGoalDist = sqrt((goalRow - neighbor.row)^2 + ...
                        (goalCol - neighbor.col)^2);
                    
                    d = obj.getDistance(currNode, neighbor);
                    
                    hScore(neighborId) = neighborGoalDist;
                    tentativeGScore = gScore(id) + d;
                    
                    if tentativeGScore < gScore(neighborId)
                        % This path to neighbor is better than any previous one. Record it!
                        parentSet{neighborId} = currNode;
                        gScore(neighborId) = tentativeGScore;
                        fScore(neighborId) = gScore(neighborId) + hScore(neighborId);
                        
                        % If not in openSet, add
                        %idx = find(cellfun( @(x) x.id == id, openSet));
                        %if isempty(idx)
                            openSet{end+1} = neighbor;
                            map(neighbor.row, neighbor.col) = 3;
                        %end
                    end
                    
                end % for each neighbor in neighbors
                map(actorRow, actorCol) = 1;
                imagesc(flipud(map(1:50, 1:50)))
                pause(0.000001)
                
            end % while cells left to explore
            warning("No valid path!")
            path = {};
        end
        
        function idx = findMinF(~, openSet, fscore)
            minF = inf;
            idx = -1;
            for i = 1 : numel(openSet)
                node = openSet{i};
                f = fscore(node.id);
                if f < minF
                    idx = i;
                    minF = f;
                end
            end
        end
        
        function d = getDistance(~, node1, node2)
            d = sqrt((node2.row - node1.row)^2 + (node2.col - node1.col)^2);
        end
        
        function neighborArray = getNeighbors(~, node, closed, map)
            
            neighborArray = {};
            
            for k = -1 : 1 : 1
                for j = -1 : 1 : 1
                    if k == 0 && j == 0
                        continue
                    else
                        row = node.row + k;
                        col = node.col + j;
                        % Check this location is in bounds
                        if (row > 0 && row < size(map, 1) && col > 0 && col < size(map, 2))
                            seen = false;
                            % Search to make sure node hasn't been seen
                            % before
                            for i = 1 : numel(closed)
                                closedNode = closed{i};
                                if (row == closedNode.row && col == closedNode.col)
                                    % Has been seen!
                                    seen = true;
                                end
                            end
                            
                            if ~seen
                                % Build neighbor node
                                newNode.row = row;
                                newNode.col = col;
                                newNode.id = sub2ind(size(map), row, col);
                                neighborArray{end+1} = newNode;
                            end
                        end
                    end
                end % j
            end % k
            
        end
    end
end

