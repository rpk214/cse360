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

% STUDENT -- create iRobot object here. 

% This implements the functionality of C's kbhit in Matlab.
% Note you need the associated my_kbhit.m file for this to work.
global kbhit;
kbhit = false;

% Initialize the arrays so that we can update them efficiently
range = zeros(181,1);
reflect = zeros(181,1);
alpha = [-45:0.5:45]'; 

% Visualize the LIDAR data
close all; 
figure('KeyPressFcn', @my_kbhit);
subplot(2,1,1);
h(1) = plot(alpha, range, 'r-', 'linewidth', 2);
subplot(2,1,2);
h(2) = plot(alpha, reflect, 'g', 'linewidth', 2);

hold on;

% Visualize camera data
figure('KeyPressFcn', @my_kbhit);
im = zeros( 480, 640, 3, 'uint8' );
im_h = image( im, 'erasemode', 'none' );
axis image;

% Start the camera
try
    start(video);
catch
    disp('Could not start video');
    SICK_LCM_Shutdown();
    return;
end

% Get scans until you hit a key.  Note, the figure has to have focus when
% you hit the key or the kbhit will be ignored. 
while ~kbhit
    [range, reflect] = SICK_LCM_GetScan();
    set(h(1), 'ydata', range);
    set(h(2), 'ydata', reflect);

    im = getdata( video, 1 );
    set( im_h, 'cdata', im );

    % STUDENT -- solve all perception and control here. 

    pause(0.01);                            % To flush the I/O buffer
end

% Clean up
SICK_LCM_Shutdown();
stop( video );
end

