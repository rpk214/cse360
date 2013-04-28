function h = plot_cov( C, x, y, varargin )
%PLOT_COV plots a covariance uncertainty ellipse
%   PLOT_COV( C, x, y ) plots the 1-sigma ellipse at position (x,y) on the
%   current axes.
%
%   PLOT_COV( C, x, y, 'Property1', 'PropertyValue1', ... )
%   sets the corresponding property values of the ellipse.  These include:
%   - 'SIGMA' which is the ellipse size in standard deviations 
%   - 'LINESTYLE', which is the line color/style string in standard "plot"
%      command format
%   - 'LINEWIDTH', which is the line thickness
%
%   H = PLOT_COV( C, x, y, ... ) returns the ellipse graphics handle. 
%

sigma=1; line_style='b-'; line_width=1;

if nargin>3
    for i=4:2:nargin
        prop = varargin{i-3};
        val = varargin{i-2};
        if strcmpi( prop, 'sigma' ), sigma=val;
        elseif strcmpi( prop, 'linestyle' ), line_style = val;
        elseif strcmpi( prop, 'linewidth' ), line_width = val;
        else error('Unrecognized argument for plot_cov'); end
    end
end
    
[v,d] = eig(C);
theta = atan(v(2,1)/v(1,1));

angle = 0:360;
a = d(1,1)^0.5*sigma;
b = d(2,2)^0.5*sigma;
p = [ a*cosd(angle); b*sind(angle) ];
p = rot2d( theta )*p;
h = plot( p(1,:)+x, p(2,:)+y, line_style, 'linewidth', line_width );


function r = rot2d(theta)
%===========================================================
% function r = rot2d(theta)
% r = [cos(theta) -sin(theta); sin(theta) cos(theta)];
%===========================================================
r = [cos(theta) -sin(theta); sin(theta) cos(theta)];