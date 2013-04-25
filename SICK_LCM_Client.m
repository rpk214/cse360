function [varargout] = SICK_LCM_Client( command, varargin )
% *************************************************************************
% function [varargout] = SICK_LCM_Client( command, varargin )
% *************************************************************************
% This is a wrapper on Mike Pollock's LCM Client code, which abstracts away
% the network communications and makes it appear as if you are accessing
% the LIDAR locally on the Client computer. 
% 
% This in turn is accessed through higher level functions which provide
% another abstraction layer. 
% 
% In practice, the students will never directly access this function.
% - DTS & JRS, 27 March 2013
% *************************************************************************

persistent agg freq initialized instantiated lc

% Instantiate Java LCM related objects
if isempty(instantiated)
    try
        lc = lcm.lcm.LCM('udpm://239.255.76.67:7667?ttl=1');
        agg = lcm.lcm.MessageAggregator();
        agg.setMaxMessages(2);
        lc.subscribe('sickData', agg);
        instantiated = true;
    catch
        error('Error instantiating Java objects');
    end
end

% Initialize the LIDAR
if strcmpi( command, 'INIT' )  && isempty(initialized)
    if nargin==2 && varargin{1}>0 && varargin{1}<75
        freq = varargin{1};
    else
        warning('No valid scan rate entered. Using 15Hz...');
        freq = 15;
    end
    initialized = true;
    
% Get a LIDAR scan
elseif strcmpi( command, 'GET_SCAN' )
    % Ensure the LIDAR is initialized
    if isempty(initialized) || ~initialized
        error('LIDAR must be initialized')
        return;
    end
    % Keep polling until we get a scan
    while 1
        ms_to_wait = 0;
        raw_msg = agg.getNextMessage(ms_to_wait);
        if ~isempty(raw_msg)
            msg = vader.sick_data(raw_msg.data);
            break;
        else
            % Wait 1/2 the scan period if there is no scan currently available
            ms_to_wait = 1000/freq/2;
        end
    end
    varargout{1} = msg.range;
    varargout{2} = msg.reflect;

% Shutting down the LIDAR
elseif strcmpi( command, 'SHUTDOWN' )
    clear lc agg freq initialized instantiated
else
% Invalid command so throw error
    error('Invalid LIDAR Command');
end
