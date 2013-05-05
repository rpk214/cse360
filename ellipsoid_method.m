function [offset,vis ] = ellipsoid_method( y_mean, cb_mean, cr_mean, c ,video )
% the size of the color model
n = 3^3;
% the number of connected pixels required to be a ball
threshold = 160; 

% figure('KeyPressFcn', @my_kbhit);
bin_im = zeros( 480, 640, 1, 'uint8' );
bin_h = image( bin_im, 'erasemode', 'none' );
axis image; colormap gray;
midpoint = [640/2, 480/2];
vis = 0;
offset = [0 0];


U = [ repmat(y_mean,1,640*480);
    repmat(cb_mean,1,640*480);
    repmat(cr_mean,1,640*480) ];

c = inv(c);

% Where the image is processed and center of object is returned in offset
% along with the visibility of the object returns [0 0] if not visible
    % Grab images and update the figure with the data
    im = getsnapshot(video);
    %     im = getdata( video );  
    y_pix = im(:,:,1);
    cb_pix = im(:,:,2);
    cr_pix = im(:,:,3);
    p = [ reshape(y_pix, 1,640*480);
          reshape(cb_pix,1,640*480);
          reshape(cr_pix,1,640*480)];
    Pu = double(p) - U;  
    Dist_pix = sum(Pu.*(c*Pu));
    Dist_im = reshape(Dist_pix,480, 640);
    bin_im = Dist_im<n;
    bin_im = 255.*bin_im;
     set( bin_h, 'cdata', bin_im ); 
    cc = bwconncomp(bin_im);

    stats = regionprops(cc,'Area','BoundingBox','Centroid');
    [biggest, idx] = max([stats.Area]);
    if biggest > threshold
        vis = 1;
        centroid = stats(idx).Centroid;
        offset = (centroid - midpoint);
    else
        vis = 0;
        offset = [0 0];
    end
    
end

