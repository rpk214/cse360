function final_proj_skel(video)
%=============================================================================
% function final_proj_skel(video)
% - Streams LIDAR video data
%
% - Note: the video object passed in to this function is the output of the
%         find_logitech_camera() function. The find_logitech_camera()
%         function should only be called once per matlab session. Also, the
%         driver for the Logitech Webcam 250 is flaky so it actually needs
%         to be unplugged and plugged back in between subsequent calls to 
%         this function.
%=============================================================================

% Set the scan rate of the SICK Client to 30 Hz
% Leave this up top as it will clear other variables when it's called
SICK_LCM_Init( 30 );

close all;

global dt;
global robot;
dt = .2;
robot = iRobotCreate(1/dt);

robot.setworkspace([-2 2 -2.5 2.5]);
makeMap();

% This implements the functionality of C's kbhit in Matlab.
% Note you need the associated my_kbhit.m file for this to work.
global kbhit;
kbhit = false;

% Start the camera
% try
%     start(video);
% catch
%     disp('Could not start video');
%     SICK_LCM_Shutdown();
%     return;
% end

lidar = lidar_config();

execute_path(lidar);
find_ball();

% Clean up
SICK_LCM_Shutdown();
% stop( video );
end

