function SICK_LCM_Shutdown()
% ************************************************************************* 
% function SICK_LCM_Shutdown()
% *************************************************************************
% Shuts down the SICK Lidar.  
% - JRS, 27 March 2013
% ************************************************************************* 

SICK_LCM_Client( 'SHUTDOWN' )
disp('SICK LCM LIDAR Shutting Down...')