function [range, reflect] = SICK_LCM_GetScan()
% ************************************************************************* 
% function [range, reflect] = SICK_LCM_GetScan()
% ************************************************************************* 
% Gets a scan from the SICK LIDAR and returns the range and reflectivity
% measurements.  Note you need to initialize the LIDAR before you can get
% scans from it.  
% - JRS, 27 March 2013
% ************************************************************************* 

[range, reflect] = SICK_LCM_Client('GET_SCAN');
