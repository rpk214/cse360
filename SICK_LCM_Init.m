function SICK_LCM_Init( freq )
% ************************************************************************* 
% function SICK_LCM_Init( freq=15 )
% *************************************************************************
% Initializes the SICK Lidar.  The update rate can be specified, and should
% be < 75 Hz.  If this is not specified, the default value will be 15 Hz.
% NOTE:  You should make this the first line of your script as it will
% call a clear and delete all the variables in the function. 
% - DTS & JRS, 27 March 2013
% ************************************************************************* 


% javaaddpath calls 'clear' under the hood, so ...
% adding paths here instead of in SICK_LCM_Client()
% to avoid clobbering the persistent variables 
javaaddpath('./lcm.jar');
javaaddpath('./vader_types.jar');

SICK_LCM_Client( 'INIT', freq );
disp('SICK LCM LIDAR Initialized...')