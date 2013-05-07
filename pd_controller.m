function pd_controller(start,points,lidar)
%PD_CONTROLLER Summary of this function goes here
%   Detailed explanation goes here

global robot;

P = [0.03 0 0; 0 0.03 0; 0 0 pi/12^2];

orientation = pi/2;
%robot.moveroomba([start orientation]);                                                  % Simulation

x = [start orientation]';
v0 = .4;
phi_max = .4;
kp = 2;
kd = 2*sqrt(kp);
current_point = start;

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
    fprintf('Rotating at point (%.2f %.2f)\n',current_point(1),current_point(2));
    fprintf('Rotating to point (%.2f %.2f)\n',points(i,1),points(i,2));
    fprintf('Current orientation is %.4f degrees\n',rad2deg(x(3)));
    fprintf('Current position estimate is (%.2f %.2f)\n',x(1),x(2));
    [x,P] = rotate(robot, current_point, points(i, :), type, x, P, lidar);
    fprintf('New value of the orientation is %.2f degrees\n\n',rad2deg(x(3)));
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
        %if exist('h','var'), delete(h); end
        %h = plot_cov( 100*P(1:2,1:2), x(1), x(2), 'linestyle', 'g', 'linewidth', 2 );
        % Determine which PD form to use based on type of path.
        if horz
            w = y_axis(points(i,2), x, kd, kp);
        elseif vert
            w = x_axis(points(i,1), x, kd, kp);
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
        [x,P] = ekf(v_robot,w_scaled,lidar,x,P);
        x_error = abs(points(i,1) - x(1));
        y_error = abs(points(i,2) - x(2));
        % Check if close to end point.
        if((horz && x_error < .05 && y_error < .25) || (vert && x_error < .25 && y_error < .05))
            done = true;
        end
    end
    % Stop and set destination as new current point.
    robot.setvel(0,0);
    current_point = points(i, :);
end

robot.setvel(0,0);

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

function [pose,P] = rotate(robot, current, goto, type, pose, P, lidar)
disp('Rotating Now')
if (strcmp(type, 'horizontal'))
    if (current(1) - goto(1) < 0)
        angle = -pose(3);
    else
        angle = pi - pose(3);
    end
elseif(strcmp(type, 'vertical'))
    if (current(2) - goto(2) < 0)
        angle = pi/2 - pose(3);
    else
        angle = -pi/2 - pose(3);
    end
end

% shift the angle into -pi to pi
while pi < angle
    angle = angle - 2 * pi;
end

while angle < -pi
    angle = angle + 2 * pi;
end

new_angle = pose(3) + angle;

if angle < 0
    while pose(3) >= new_angle + 0.2618
        robot.setvel(0,-pi/8);
        [pose,P] = ekf(0,-pi/8,lidar,pose,P);
    end
else
    while pose(3) <= new_angle - 0.2618
        robot.setvel(0,pi/8);
        [pose,P] = ekf(0,pi/8,lidar,pose,P);
    end
end

function w = y_axis(yd, pose, kd, kp)

e = yd - pose(2);
w = -kd * tan(pose(3)) + (kp * e)/(.5*cos(pose(3)));

function w = x_axis(xd, pose, kd, kp)

e = xd - pose(1);
w = kd * cot(pose(3)) - (kp * e)/(.5*sin(pose(3)));


