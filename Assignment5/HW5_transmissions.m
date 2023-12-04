close all
clear
clc

global THICKNESS LENGTH HEIGHT WIDTH
THICKNESS = 0.025;
LENGTH = 0.61;
HEIGHT = 0.65;
WIDTH = 0.42;

%% main

% Obstacle creation (walls and secondary shaft)
[walls, secondary_shaft] = create_obstacles();
obstacles = {walls, secondary_shaft};
colors = [[0.75 0 0];[0 0 0.75]];

primary_shaft = create_shaft();


% Figure parameters
figure
xlim([0, LENGTH+THICKNESS])
ylim([-WIDTH/2, WIDTH/2])
zlim([0, 1])
xlabel("x axis")
ylabel("y axis")
zlabel("z axis")
hold on
view([10 20.75])

draw_obstacles(obstacles,colors);

show(primary_shaft,"Collisions","on","Frames","off");
showdetails(primary_shaft)

%% Helper functions

function [walls, shaft] = create_obstacles()

    walls = create_walls();
    shaft = create_secondary_shaft();

end

function shaft = create_shaft()
    % Primary shaft is created as a robot (rigid body tree)

    % Initialize robot
    shaft = rigidBodyTree("DataFormat", "column");
    base = shaft.Base;
    
    % Create rigid bodies
    c1 = rigidBody("c1");
    c2 = rigidBody("c2");
    c3 = rigidBody("c3");
    c4 = rigidBody("c4");
    c5 = rigidBody("c5");
    c6 = rigidBody("c6");
    c7 = rigidBody("c7");
    c8 = rigidBody("c8");
    c9 = rigidBody("c9");
    cA = rigidBody("cA");
    cB = rigidBody("cB");
    
    % Define joints
    jntBase = rigidBodyJoint("base_joint","fixed");
    jnt1 = rigidBodyJoint("jnt1t2","fixed");
    jnt2 = rigidBodyJoint("jnt2t3","fixed");
    jnt3 = rigidBodyJoint("jnt3t4","fixed");
    jnt4 = rigidBodyJoint("jnt4t5","fixed");
    jnt5 = rigidBodyJoint("jnt5t6","fixed");
    jnt6 = rigidBodyJoint("jnt6t7","fixed");
    jnt7 = rigidBodyJoint("jnt7t8","fixed");
    jnt8 = rigidBodyJoint("jnt8t9","fixed");
    jnt9 = rigidBodyJoint("jnt9tA","fixed");
    jntA = rigidBodyJoint("jntAtB","fixed");
    
    % Definition of collision geometries
    cylinders = {struct("radius",0.072/2,"length",0.04,"origin",[0.04/2; 0; 0]),
                 struct("radius",0.18/2,"length",0.016,"origin",[0.016/2; 0; 0]),
                 struct("radius",0.239/2,"length",0.06,"origin",[0.06/2; 0; 0]),
                 struct("radius",0.18/2,"length",0.066,"origin",[0.066/2; 0; 0]),
                 struct("radius",0.212/2,"length",0.05,"origin",[0.05/2; 0; 0]),
                 struct("radius",0.18/2,"length",0.016,"origin",[0.016/2; 0; 0]),
                 struct("radius",0.240/2,"length",0.05,"origin",[0.05/2; 0; 0]),
                 struct("radius",0.239/2,"length",0.06,"origin",[0.06/2; 0; 0]),
                 struct("radius",0.18/2,"length",0.016,"origin",[0.016/2; 0; 0]),
                 struct("radius",0.260/2,"length",0.05,"origin",[0.05/2; 0; 0]),
                 struct("radius",0.072/2,"length",0.236,"origin",[0.236/2; 0; 0]),};
    
    bodies = {base, c1, c2, c3, c4, c5, c6, c7, c8, c9, cA, cB};
    joints = {[], jntBase, jnt1, jnt2, jnt3, jnt4, jnt5, jnt6, jnt7, jnt8, jnt9, jntA};
    
    % bodies = {base, c1, c2, c3};
    % joints = {[], jntBase, jnt1, jnt2};
    
    % The pose of the base controls the movement of the whole shaft
    pose = [roty(90) [0.185; 0; 0.522]; 0 0 0 1];
    setFixedTransform(jntBase,pose);
    
    % Attach collision cylinders and set transforms between cylinders
    cylinders_number = length(cylinders);
    for i = 1:cylinders_number
        cylinder_properties = cylinders{i};
    
        % Create collision geometry
        cylinder = collisionCylinder(cylinder_properties.radius, cylinder_properties.length);
        % pose = [eye(3) cylinder_properties.origin; 0 0 0 1];
        pose = [eye(3) [0;0;0]; 0 0 0 1];
        cylinder.Pose = pose;
    
        % Set the offset of the cylinder position 
        if i >1
            setFixedTransform(joints{i+1},trvec2tform([0 0 cylinders{i-1}.origin(1)+cylinder_properties.origin(1)]));    
        end
    
        % Attach the collision geometry to the part of the robot
        addCollision(bodies{i+1}, cylinder)
    
        % Attach the jpoin to the part of the robot
        bodies{i+1}.Joint = joints{i+1};
    
        % Attach the part to the robot itself
        addBody(shaft,bodies{i+1},bodies{i}.Name)
    end
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
        % disp("i:" + i)
        for j = 1:elements_number
            % disp("j:" + j)
            element = obstacle(j);
            [~,patchObj] = show(element);
            patchObj.FaceColor = element_color;
            patchObj.EdgeColor = 'none';
        end
    end
end
