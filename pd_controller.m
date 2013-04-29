function pd_controller(points)
%PD_CONTROLLER Summary of this function goes here
%   Detailed explanation goes here

close all;

robot = iRobotCreate(5);

robot.setworkspace([-2 2 -2.5 2.5]);

orientation = rand(1) * (2 * pi/3) - (pi/3);
robot.moveroomba([-1.25 -1 orientation]);

axis equal;
plot([-1.25 1.25], [1 1], 'linewidth', 2);
plot([-1.25 1.25], [-1 -1], 'linewidth', 2);
plot([0 0], [-2.25 2.25], 'linewidth', 2);
plot([-1.25 1.25], [-1 1], 'linewidth', 2);

pose = robot.getpose();
v0 = .5;
phi_max = .5;
kd = 2.4495;
kp = 1.5;
current_point = [-1.25 -1];

dimensions = size(points);

for i = 1:dimensions(1)
    done = false;
    horz = false;
    vert = false;
    diag = false;
    circle = false;
    % Determine if path is horizontal, vertical, diagonal, or along a
    % circle.
    type = type_of_path(current_point, points(i, :));
    % Make sure robot is facing correct direction.
    rotate(robot, current_point, points(i, :), type);
    % Assign booleans based on type of path.
    if (strcmp(type ,'vertical'))
        vert = true;
    elseif (strcmp(type, 'horizontal'))
        horz = true;
    elseif(strcmp(type,'diagonal'))
        diag = true;
    else
        circle = true;
    end
    while ~done
        % Determine which PD form to use based on type of path.
        if horz
            w = y_axis(points(i,2), pose, kd, kp);
        elseif vert
            w = x_axis(points(i,1), pose, kd, kp);
        end
                
        % Move robot.
        v_left = v0 - .3 * w;
        v_right = v0 + .3 * w;
        
        max_v = max([abs(v_left) abs(v_right)]);
        scale = max_v/phi_max;
        v_left_scaled = v_left/scale;
        v_right_scaled = v_right/scale;
        w_scaled = w/scale;
        
        v_robot = (v_left_scaled + v_right_scaled)/2;
        robot.setvel(v_robot, w_scaled);
        pose = robot.getpose();
        x_error = abs(points(i,1) - pose(1));
        y_error = abs(points(i,2) - pose(2));
        % Check if close to end point.
        if (x_error < .05 && y_error < .05)
            done = true;
        end
    end
    % Stop and set destination as new current point.
    robot.setvel(0,0);
    current_point = points(i, :);
end

robot.setvel(0,0);
disp('Hit any key to continue...');
pause;

function type = type_of_path(current, goto)
p1 = [-1.25, -1];
p7 = [0,0];
p4 = [1.25, 1];
if (current(1) - goto(1) == 0)
    type = 'vertical';
elseif (current(2) - goto(2) == 0)
    type = 'horizontal';
elseif ((isequaln(current, p1) || isequaln(current, p7) || isequaln(current, p4)) ... 
    && (isequaln(goto, p1) || isequaln(goto, p7) || isequaln(goto, p4)))
    type = 'diagonal';
else
    type = 'circle';
end

function rotate(robot, current, goto, type)
pose = robot.getpose();
if (strcmp(type, 'horizontal'))
    if (current(1) - goto(1) < 0)
        robot.rotate(-pose(3));
    else
        robot.rotate(pi - pose(3));
    end
elseif(strcmp(type, 'vertical'))
    if (current(2) - goto(2) < 0)
        robot.rotate(pi/2 - pose(3));
    else
        robot.rotate(-pi/2 - pose(3));
    end
end

function w = y_axis(yd, pose, kd, kp)

e = yd - pose(2);
w = -kd * tan(pose(3)) + (kp * e)/(.5*cos(pose(3)));

function w = x_axis(xd, pose, kd, kp)

e = xd - pose(1);
w = kd * cot(pose(3)) - (kp * e)/(.5*sin(pose(3)));


