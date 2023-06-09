function [z_beacon, lastBeaconTime] = getBeacon(dataStore, beacon)
% GETBEACON: given beacon data and beacon info, compute the measurement
% data z_beacon
%
%   INPUTS:
%       dataStore.beacon_data       k-by-6 [time, time_delay, id, x, y, theta] in robot
%                                   frame
%       dataStore.lastBeaconTime    time stamp of the last beacon (used to determine whether we read the new beacon)
%       beacon                      n-by-3 [x, y, id] in global
%
%   OUTPUTS:
%       z_beacon        2n-by-1 [x1; y1; ...] in global
%       lastBeaconTime  updated last beacon time
%
n = size(beacon, 1);
z_beacon = NaN(2*n,1);
beacon_data = dataStore.beacon;
lastBeaconTime = dataStore.lastBeaconTime;

if size(beacon_data, 1) >= 1
    for i=size(beacon_data, 1):-1:1
        if isnan(lastBeaconTime) || ((beacon_data(end,1) ~= lastBeaconTime) && (beacon_data(i, 1) == beacon_data(end, 1)))
            id = beacon_data(i, 3);
            idx = find(beacon(:, 3) == id);
            z_beacon(idx * 2 - 1:idx * 2) = beacon_data(i, 4:5).';
        else
            break;
        end
    end
    lastBeaconTime = beacon_data(end, 1);
end
end