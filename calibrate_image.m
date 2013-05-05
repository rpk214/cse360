function [ y_mean cb_mean cr_mean c ] = calibrate_image( )
% this function returns the color model of an object selected
% from the camera
close all;

% This implements the functionality of C's kbhit in Matlab.
% Note you need the associated my_kbhit.m file for this to work.
global kbhit;
kbhit = false;

% Set up the figure parameters
figure('KeyPressFcn', @my_kbhit);
im = zeros( 480, 640, 3, 'uint8' );
im_h = image( im, 'erasemode', 'none' );
axis image;

% Get the video source
video = find_logitech_camera ;
% Start the camera
start(video);

while ~kbhit
    % Grab images and update the figure with the data
    im = getdata( video, 1 );
    % This next line is just for illustration purposes to see an RGB image
    % You will comment it out for logging images and the actual segmentation
    %im = ycbcr2rgb( im );
    set( im_h, 'cdata', im ); 
end

mask = roipoly(im);
y_pix = im(:,:,1);
cb_pix = im(:,:,2);
cr_pix = im(:,:,3);

y_ball = y_pix(mask);
cb_ball = cb_pix(mask);
cr_ball = cr_pix(mask);

y_mean = mean(y_ball);
cb_mean = mean(cb_ball);
cr_mean = mean(cr_ball);

c = cov(double([y_ball cb_ball cr_ball]));

close all;

% Cleanup
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
set( src, 'FrameRate', '15' );
% This is for all cameras
set( video, 'TriggerRepeat', Inf );
end



