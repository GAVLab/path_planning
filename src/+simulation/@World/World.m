classdef World < handle
    %WORLD Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Objects
        actors_ % : Cell array of Actor objects
        lostActors_ % : Cell array of Actors that have left the map
        
        map_
        
        time_
        
        occupancy_grid_ % : (numGridCols x numGridRows) array
        %{
        Important note about occupancy grids -
        The cell at (i, j) is really a (resolution, resolution) square
        centered at ((i - 0.5)*resolution, (j - 0.5)*resolution)
        Because of this, grid cell (i, j) contains:
        x \in (i-1)*resolution : i * resolution
        y \in (j-1)*resolution : j * resolution
        %}
        
        % Parameters
        gridResolution_ % : pixels per meter
        numGridRows_
        numGridCols_
        gridHeight_ % height of grid in meters
        gridWidth_ % width of grid in meters
        
    end
    
    methods (Access = public)
        function obj = World(width, height, resolution, actors)
            %WORLD Construct an instance of this class
            %   Detailed explanation goes here
            
            obj.gridHeight_ = width;
            obj.gridWidth_ = height;
            obj.gridResolution_ = resolution;
            
            obj.numGridRows_ = obj.gridHeight_ * obj.gridResolution_;
            obj.numGridCols_ = obj.gridWidth_ * obj.gridResolution_;
            
            %obj.actors_ = actors;
            obj.actors_ = cell(100,1);
            
            for i = 1 : length(actors)
                actor = actors{i};
                id = actor.id_;
                obj.actors_{id} = actor;
            end
            
            obj.map_ = zeros(obj.numGridRows_, obj.numGridCols_, 'uint8');
            
            % A note: in this grid, X = right, Y = down. Before visualizing
            % you should flip it upside down so it makes sense.
            obj.occupancy_grid_ = obj.buildOccupancyGrid();
            
            obj.time_ = 0;
        end
        
        function step(obj, dt)
            %STEP Update the position of every actor
            %    For every actor, calculate control input and propagate
            %    state forward
            
            obj.time_ = obj.time_ + dt;
            
            updatedActors = cell(size(obj.actors_));
            
            for i = 1 : length(obj.actors_)
                if isempty(obj.actors_{i})
                    continue
                end
                currActor = obj.actors_{i};
                currActor = currActor.calculateControlInput(obj, dt);
                newActor = currActor.applyMotionModel(dt);
                
                % TODO: Collision checking
                newPosition = newActor.state_.position;
                newX = newPosition(1);
                newY = newPosition(2);
                
                
                % Check if actor has moved outside of map
                if newX < 0 || newX > obj.gridWidth_ || newY < 0 || newY > obj.gridHeight_
                    % Do nothing - don't add it back to the new state
                    disp(['Removing: ', num2str(newActor.id_)]);
                    obj.lostActors_{end+1} = newActor;
                else
                    updatedActors{currActor.id_} = newActor;
                end
                
                % Maybe we don't do collision checking and just track the
                % distance between the actors. If it goes below a threshold
                % we just say they've collided and ignore data after that.
                % Realistic collision handling seems outside the scope of
                % this project atm.
            end
            
            obj.actors_ = updatedActors;
            obj.occupancy_grid_ = obj.buildOccupancyGrid();
        end
        
        function actor = getActorById(obj, id)
            actor = -1;
            if ~isempty(obj.actors_{id})
                actor = obj.actors_{id};
            end
        end
        
        function grid = buildOccupancyGrid(obj)
            grid = obj.map_;
            
            for i = 1 : length(obj.actors_)
                if isempty(obj.actors_{i})
                    continue
                end
                actor = obj.actors_{i};
                position = actor.state_.position;
                [row, col] = obj.coordsToIdx(position);
                grid(row, col) = 255;
            end
        end
        
        function actorList = removeActor(~, actorList, actor)
            %REMOVEACTOR
            id = actor.id_;
            
            if ~isempty(actorList{id})
                actorList(id) = [];
            end
        end
        
        function [row, col] = coordsToIdx(obj, coords)
            x = coords(1);
            y = coords(2);
            
            row = ceil(y * obj.gridResolution_);
            col = ceil(x * obj.gridResolution_);
        end
        
        function [x, y] = idxToCoords(obj, idxPair)
            % (1,1) -> 0.25, 0.25
            % (2,2) -> 0.75, 0.75
            % (3,3) -> 1.25, 1.25
           row = idxPair(1);
           col = idxPair(2);
           
           x = (col - (1/obj.gridResolution_)) / obj.gridResolution_;
           y = (row - (1/obj.gridResolution_)) / obj.gridResolution_;
           
           
        end
    end
end

