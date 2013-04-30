function [x,P] = ekf(v,omega,lidar,x,P)

% You should only add code where you see
% *** ADD NECESSARY CODE HERE ***
% You should NOT change any parameters/values that have already been set. 

global kbhit;
global dt;
global robot;

Q = [   .01^2   0           ;
        0       (pi/60)^2   ];
R = [   .05^2   0           ;
        0       (pi/60)^2   ];

% Time Update
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
% [rho, alpha] = scan_real_lidar();
H = [   (x(1) - lidar(1))/(sqrt((x(1)-lidar(1))^2 + (x(2)-lidar(2))^2))         (x(2) - lidar(2))/(sqrt((x(1)-lidar(1))^2 + (x(2)-lidar(2))^2))     0   ;
        (-(x(2)-lidar(2))/((x(1)-lidar(1))^2 + (x(2)-lidar(2))^2))              ((x(1)-lidar(1))/((x(1)-lidar(1))^2 + (x(2)-lidar(2))^2))           0   ];
K = P*H'/(H*P*H' + eye(2)*R*eye(2)');

x = x + K*([rho; alpha] - [sqrt((x(1)-lidar(1))^2 + (x(2)-lidar(2))^2); atan2(x(2) - lidar(2),(x(1)-lidar(1))) - lidar(3)]);
P = (eye(3) - K*H)*P;

function [rho, alpha] = scan_lidar( lidar, robot, R )
pose = robot.getpose();
rho = sqrt(((lidar(1)-pose(1))^2 + ((lidar(2)-pose(2))^2))) + R(1,1)^.5 * randn;
alpha = atan2(pose(2) - lidar(2),(pose(1)-lidar(1))) - lidar(3) + R(2,2)^.5 * randn;

function [rho, alpha] = scan_real_lidar()
range = zeros(181,1);
reflect = zeros(181,1);
alpha = [-45:0.5:45]';
[range, reflect] = SICK_LCM_GetScan();
valid = (reflect > 200);
alpha_valid = alpha(valid);
range_valid = range(valid);
alpha = sum(alpha_valid) / length(alpha_valid);
rho = sum(range_valid) / length(range_valid);