close all
clear
clc

global THICKNESS LENGTH HEIGHT WIDTH
THICKNESS = 0.025;
LENGTH = 0.61;
HEIGHT = 0.65;
WIDTH = 0.42;

%% main

[walls, secondary_shaft] = create_obstacles();
obstacles = {walls, secondary_shaft};
colors = [[0.75 0 0];[0 0.75 0]];

% Figure parameters
figure
xlim([0, LENGTH+THICKNESS])
ylim([-WIDTH/2, WIDTH/2])
zlim([0, HEIGHT])
xlabel("x axis")
ylabel("y axis")
zlabel("z axis")
hold on
view([10 20.75])

draw_obstacles(obstacles,colors);

%% Helper functions

function [walls, shaft] = create_obstacles()

    walls = create_walls();
    shaft = create_secondary_shaft();

end

function walls = create_walls()
    global THICKNESS LENGTH HEIGHT WIDTH

    % width is defined along the y axis
    % height is defined along the z axis
    boxes = {
             % Wall on the right
             struct("width",WIDTH,"height",0.400,"origin",[THICKNESS/2 + LENGTH + THICKNESS/2;0;0.400/2]),
             struct("width",(WIDTH-0.16)/2,"height",0.16,"origin",[THICKNESS/2 + LENGTH + THICKNESS/2; 0.16/2 + (WIDTH-0.16)/4; 0.400 + 0.16/2]),
             struct("width",(WIDTH-0.16)/2,"height",0.16,"origin",[THICKNESS/2 + LENGTH + THICKNESS/2; -0.16/2 - (WIDTH-0.16)/4; 0.400 + 0.16/2]),
             struct("width",WIDTH,"height",0.09,"origin",[THICKNESS/2 + LENGTH + THICKNESS/2; 0; 0.400 + 0.16 + 0.09/2]),
             
             % Wall on the left
             struct("width",WIDTH,"height",0.400,"origin",[THICKNESS/2;0;0.400/2]),
             struct("width",(WIDTH-0.16)/2,"height",0.16,"origin",[THICKNESS/2; 0.16/2 + (WIDTH-0.16)/4; 0.400 + 0.16/2]),
             struct("width",(WIDTH-0.16)/2,"height",0.16,"origin",[THICKNESS/2; -0.16/2 - (WIDTH-0.16)/4; 0.400 + 0.16/2]),
             struct("width",WIDTH,"height",0.09,"origin",[THICKNESS/2; 0; 0.400 + 0.16 + 0.09/2]),
            };
    
    walls = [];
    boxes_number = length(boxes);
    for i = 1:boxes_number
        box_properties = boxes{i};
    
        box = collisionBox(THICKNESS, box_properties.width, box_properties.height);
        pose = [eye(3) box_properties.origin; 0 0 0 1];
        box.Pose = pose;
    
        walls = [walls, box];
    end
end

function secondary_shaft = create_secondary_shaft()
    global THICKNESS LENGTH HEIGHT WIDTH

    cylinders = {struct("radius",0.072/2,"length",0.051,"origin",[THICKNESS/2 + 0.051/2;0;0.25]),
                 struct("radius",0.2/2,"length",0.05,"origin",[THICKNESS/2 + 0.051 + 0.05/2;0;0.25]),
                 struct("radius",0.072/2,"length",0.076,"origin",[THICKNESS/2 + 0.051 + 0.05 + 0.076/2;0;0.25]),
                 struct("radius",0.16/2,"length",0.05,"origin",[THICKNESS/2 + 0.051 + 0.05 + 0.076 + 0.05/2;0;0.25]),
                 struct("radius",0.072/2,"length",0.016,"origin",[THICKNESS/2 + 0.051 + 0.05 + 0.076 + 0.05 + 0.016/2;0;0.25]),
                 struct("radius",0.247/2,"length",0.05,"origin",[THICKNESS/2 + 0.051 + 0.05 + 0.076 + 0.05 + 0.016 + 0.05/2;0;0.25]),
                 struct("radius",0.279/2,"length",0.05,"origin",[THICKNESS/2 + 0.051 + 0.05 + 0.076 + 0.05 + 0.016 + 0.05 + 0.05/2;0;0.25]),
                 struct("radius",0.072/2,"length",0.182,"origin",[THICKNESS/2 + 0.051 + 0.05 + 0.076 + 0.05 + 0.016 + 0.05 + 0.05 + 0.182/2;0;0.25]),
                 struct("radius",0.280/2,"length",0.05,"origin",[THICKNESS/2 + 0.051 + 0.05 + 0.076 + 0.05 + 0.016 + 0.05 + 0.05 + 0.182 + 0.05/2;0;0.25]),
                 struct("radius",0.072/2,"length",0.033,"origin",[THICKNESS/2 + 0.051 + 0.05 + 0.076 + 0.05 + 0.016 + 0.05 + 0.05 + 0.182 + 0.05 + 0.033/2;0;0.25])};
    
    secondary_shaft = [];
    cylinders_number = length(cylinders);
    for i = 1:cylinders_number
        cylinder_properties = cylinders{i};
    
        cylinder = collisionCylinder(cylinder_properties.radius, cylinder_properties.length);
        pose = [roty(90) cylinder_properties.origin; 0 0 0 1];
        cylinder.Pose = pose;
    
        secondary_shaft = [secondary_shaft, cylinder];
    end
end

function draw_obstacles(obstacles_array, colors_array)

    % Obstacles is a cell array containing all obstacles
    % Each obstacle is an array of collision objects (boxes, cylinders)
    % Colors array is just an array of RGB colors
    obstacle_number = length(obstacles_array);
    for i = 1:obstacle_number
        obstacle = obstacles_array{i};
        elements_number = length(obstacle);
        element_color = colors_array(i,:);
        disp("i:" + i)
        for j = 1:elements_number
            disp("j:" + j)
            element = obstacle(j);
            [~,patchObj] = show(element);
            patchObj.FaceColor = element_color;
            patchObj.EdgeColor = 'none';
        end
    end
end
