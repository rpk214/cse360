function  find_ball()%robot )
% % create robot object iRobotCreate(sim, COM, update)
robot = iRobotCreate(0,6,5);

% the color model to be programmed base on the model created using the 
% calibrate image function on competition day
y_mean =  221.9598;
cb_mean = 55.2268;
cr_mean =   91.3736;
c = [634.4013  482.9924  629.1323
    482.9924  490.9402  602.8348
    629.1323  602.8348  761.6641];

% Setup the video from here and pass the video object to ellipsoid_method
video = find_logitech_camera;
% Start the camera
start(video);
 
% get a frame and set visibility of ball
[ center, visible ] = ellipsoid_method(y_mean, cb_mean, cr_mean, c,video);
i=0;


while true
% if visble got to the ball else rotate 45degrees and look again
    if visible
        x = center(1);
        rotation_angle = x*0.03;
        rotation_angle = rotation_angle * (2*pi/360);
            robot.rotate(rotation_angle);
            robot.forward(.3);
%         pause(1);
%         display('ball');
    else
          robot.rotate(pi/3);
%         display('noball');
%         pause(1);
    end
    [ center, visible ] = ellipsoid_method(y_mean, cb_mean, cr_mean,c,video);
    if i == 10
        flushdata(video);
        i = 0;
    end
    i = i + 1;
end
    
 stop( video );
 delete( video );
 clear video;

end

function video = find_logitech_camera
%==========================================================================
% Setting up the camera parameters
% Reset the camera regardless if we need to or not
imaqreset();
% This is to find the Logitech camera
info = imaqhwinfo('winvideo');
num_devices = length(info.DeviceIDs);
camera_found = false;
for dev_num = 1:num_devices
    info = imaqhwinfo('winvideo', dev_num);
    if strcmpi( info.DeviceName, 'Logitech Webcam 250' )
        video = videoinput( 'winvideo', dev_num, 'RGB24_640x480' );
        camera_found = true;
        disp('Found Logitech Webcam 250...');
    end
end
% We could not find the Logitech web cam so something is amiss
if ~camera_found
    error('Camera could not be found! Ensure its plugged in properly!');
end

% Set camera parameters.  Note these settings are for the HP laptop to try
% and minimize the impact of background lighting compensation.  You can
% try to tune these for the Logitech camera, but I think thesy are probably
% fine as is.
src = getselectedsource(video);
set( src, 'BacklightCompensation', 'off');
set( src, 'WhiteBalanceMode', 'manual');
set( src, 'FrameRate', '3' );
% This is for all cameras
set( video, 'TriggerRepeat', Inf );

end


