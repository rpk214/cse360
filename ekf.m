function hwk4

% You should only add code where you see
% *** ADD NECESSARY CODE HERE ***
% You should NOT change any parameters/values that have already been set. 

% Close any/all open figures, and open a new one associated with kbhit()
close all;

% Do not change these variables
global kbhit;
kbhit = false;
dt = 0.2;
v = 0.4;
omega = pi/10;
lidar_coords = [0.1 -0.1 -0.1 0.1; 0.15 0.15 -0.15 -0.15];

% Initialize the pose of the LIDAR.  
lidar = [0.2*rand; 0.2*rand; pi/12*randn+pi/2];

% Initialize the position and orientation covariance uncertainty.  
P = [0.01 0 0; 0 0.01 0; 0 0 pi/4^2];
x = [0;0.5;0] + rand(3,1).*sqrt([P(1,1); P(2,2); P(3,3)]);

% Create iRobotCreate object with update rate 5 Hz.  
robot = iRobotCreate(1/dt);
% Sets the bounds for the GUI.
robot.setworkspace([-2 2 -0.5 3.5]);
% Set the initial position and orientation
robot.moveroomba( x );
axis equal;

% This is to register kbhit functionality with the figure
iptaddcallback(gcf, 'KeyPressFcn', @my_kbhit);

% Plot the position of the LIDAR.  Do not change this.  
lidar_coords = rot2d(lidar(3))*lidar_coords + repmat([lidar(1); lidar(2)], 1, 4);
lidar_h = fill(lidar_coords(1,:), lidar_coords(2,:), 'b');

Q = [   .01^2   0           ;
        0       (pi/60)^2   ];
R = [   .05^2   0           ;
        0       (pi/60)^2   ];
I = eye(3);
V = eye(2);
while ~kbhit()
    % Time Update
    robot.setvel(v + randn*sqrt(Q(1,1)),omega + randn*sqrt(Q(2,2)));
    x = x + [v*cos(x(3))*dt; v*sin(x(3))*dt; omega*dt];
    A = [   1   0   -v*dt*sin(x(3)) ;
            0   1   v*dt*cos(x(3))  ;
            0   0   1               ];
    W = [   cos(x(3))*dt    0   ;
            sin(x(3))*dt    0   ;
            0               dt  ];
    
    P = A*P*A' + W*Q*W';

    % Measurement Update
    [rho, alpha] = scan_lidar( lidar, robot, R );
    H = [   (x(1) - lidar(1))/(sqrt((x(1)-lidar(1))^2 + (x(2)-lidar(2))^2))         (x(2) - lidar(2))/(sqrt((x(1)-lidar(1))^2 + (x(2)-lidar(2))^2))     0   ;
            (-(x(2)-lidar(2))/((x(1)-lidar(1))^2 + (x(2)-lidar(2))^2))              ((x(1)-lidar(1))/((x(1)-lidar(1))^2 + (x(2)-lidar(2))^2))           0   ];
    K = P*H'/(H*P*H' + V*R*V');
    
    x = x + K*([rho; alpha] - [sqrt((x(1)-lidar(1))^2 + (x(2)-lidar(2))^2); atan2(x(2) - lidar(2),(x(1)-lidar(1))) - lidar(3)]);
    P = (I - K*H)*P;
    
    % Plot P after the measurement update (should be smaller)
    if exist('h','var'), delete(h); end
    h = plot_cov( 100*P(1:2,1:2), x(1), x(2), 'linestyle', 'g', 'linewidth', 2 );
    
    % We need this to flush the I/O so it plots nicely
    pause(0.01);
end

disp('Simulation finished.  Save your figures if necessary and hit any key to continue...')
pause();


% Implement this function IAW B.2.c of the assignment
function [rho, alpha] = scan_lidar( lidar, robot, R )
pose = robot.getpose();
rho = sqrt(((lidar(1)-pose(1))^2 + ((lidar(2)-pose(2))^2))) + R(1,1)^.5 * randn;
alpha = atan2(pose(2) - lidar(2),(pose(1)-lidar(1))) - lidar(3) + R(2,2)^.5 * randn;


% function R = rot2d(theta)
% - Generate a 2D rotation matrix for a given angle theta
function R = rot2d(theta)
R = [cos(theta) -sin(theta); sin(theta) cos(theta)];

