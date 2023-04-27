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
%       z_beacon        2n-by-1 [x1; y1; ...] in global
%
n = size(beacon, 1);
z_beacon = NaN(2*n,1);
if size(beacon_data, 1) >= 1

    last_time = beacon_data(end, 1);
    for i=size(beacon_data, 1):-1:1
        if beacon_data(i, 1) == last_time
            id = beacon_data(i, 3);
            idx = find(beacon(:, 3) == id);
%             disp('idx')
%             idx
%             disp('z_beacon')
%             z_beacon(idx * 2 - 1:idx * 2)
%             size(z_beacon(idx * 2 - 1:idx * 2))
%             size(beacon_data(i, 4:5).')
            z_beacon(idx * 2 - 1:idx * 2) = beacon_data(i, 4:5).';
        else
            break;
        end
    end
end

end