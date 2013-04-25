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
set( src, 'FrameRate', '15' );
% This is for all cameras
set( video, 'TriggerRepeat', Inf );
