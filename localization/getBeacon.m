function z_beacon = getBeacon(beacon_data, beacon)
% GETBEACON: given beacon data and beacon info, compute the measurement
% data z_beacon
%
%   INPUTS:
%       beacon_data     k-by-6 [time, time_delay, id, x, y, theta] in robot
%                       frame
%       beacon          n-by-3 [x, y, id] in global
%
%   OUTPUTS:
%       z_beacon        1-by-2n [x1, y1, ..., xn, yn]
%
n = size(beacon, 1);
z_beacon = NaN(1, 2*n);
last_time = beacon_data(end, 1);
for i=size(beacon_data, 1):-1:1
    if beacon_data(i, 1) == last_time
        id = beacon_data(i, 3);
        idx = find(beacon(:, 3) == id);
        z_beacon(idx * 2 - 1:idx * 2) = beacon_data(i, 4:5);
    else
        break;
    end
end

end