%% Main
clear
close all

% Define the map dimensions
x_size = 128;
y_size = 128;

% Define the number of obstacles based on the defined map coverage
coverage = 0.1;
number_of_obstacles = coverage*x_size*y_size/4;

% Create empty map
map = binaryOccupancyMap(x_size, y_size, 1);

% Add obstacles
map = populate_obstacles(map, number_of_obstacles);

map.show()

%% Functions

function populated_map = populate_obstacles(empty_map, number_of_obstacles)

    for obstacle_count = 1:number_of_obstacles
        % Retry until obstacle is succesfully populated
        success = false;
        while ~success
            success = true;
    
            % Generate random locations
            obstacle_location = rand(1,2).*[empty_map.GridSize(1), empty_map.GridSize(2)];
            obstacle_location = round(obstacle_location);
            obs_x = obstacle_location(1);
            obs_y = obstacle_location(2);
        
            % Check if there is already an obstacle in this location
            if ~empty_map.getOccupancy([obs_x, obs_y])
            
                % Random selection of the type of tetromino
                obstacle_type = randi(4,1);
    
                % Generate the coordinates for the parts that compose the
                % obstacle
                switch obstacle_type
                    case 1
                        obstacle_points = [[obs_x, obs_y];
                                            [obs_x, obs_y+1];
                                            [obs_x, obs_y+2];
                                            [obs_x, obs_y+3]];
                    case 2
                        obstacle_points = [[obs_x, obs_y];
                                            [obs_x+1, obs_y];
                                            [obs_x+1, obs_y-1];
                                            [obs_x+1, obs_y-2]];
                    case 3
                        obstacle_points = [[obs_x, obs_y];
                                            [obs_x, obs_y-1];
                                            [obs_x+1, obs_y-1];
                                            [obs_x+1, obs_y-2]];
                    case 4
                        obstacle_points = [[obs_x, obs_y];
                                            [obs_x, obs_y-1];
                                            [obs_x, obs_y-2];
                                            [obs_x-1, obs_y-1]];
                end
    
                % Check if the blocks of the obstacle are within the map
                if any(obstacle_points(:,2) > empty_map.GridSize(2)) || any(obstacle_points(:,1) > empty_map.GridSize(1))
                    success = false;
                end
    
                if any(obstacle_points(:,2) < 0) || any(obstacle_points(:,1) < 0)
                    success = false;
                end
    
                % Check that the locations dont contain an obstacle already
                if any(empty_map.getOccupancy(obstacle_points))
                    success = false;
                end
    
                % Populate the obstacle if all checks passed
                if success
                    empty_map.setOccupancy(obstacle_points, 1);
                end
            else
                success = false;
            end
        end
    end

    populated_map = empty_map;
end