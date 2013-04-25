function im = grab_rgb_image(video)
%=======================================================================
% function im = grab_rgb_image()
% - Streams images to a figure. 
% - A video object for the Logitech Webcam 250 must be passed in.
% - When a key is hit, the last streamed image is returned.  
% - Useful for snapping a specific image
%=======================================================================
close all;

% This implements the functionality of C's kbhit in Matlab.
global kbhit;
kbhit = false;

% Set up the figure parameters
figure('KeyPressFcn', @my_kbhit);
im = zeros( 480, 640, 3, 'uint8' );
im_h = image( im, 'erasemode', 'none' );
axis image;

% Start the camera
start(video);

while ~kbhit
    % Grab images and update the figure with the data
    im = getdata( video, 1 );
    set( im_h, 'cdata', im ); 
end

% Cleanup
stop( video );
