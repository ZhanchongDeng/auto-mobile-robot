function [mu_next, sigma_next] = EKF(mu, sigma, u, z_depth, z_beacon, ...
    dynamics_model, dynamics_jacobian, R, sensor_model, sensor_jacobian, Q, errThreshold)
% EKF: One step EKF(Extended Kalman Filter), predict and update
%
% INPUT:
%   mu                  -   3 x 1 mean of the state [x; y; theta]
%   sigma               -   3 x 3 covariance of the state
%   u                   -   control input [d; phi]
%   z_depth             -   depth measurement 1 x K
%   z_beacon            -   beacon measurement 1 x 2N (N is the total number of beacons), non-present entries are NaN
%   dynamics_model      -   g(mu, u) -> next mu
%   dynamics_jacobian   -   g_jac(mu, u) -> 3 x 3 Jacobian of the dynamics
%   R                   -   3 x 3 covariance of the control noise
%   sensor_model        -   h(mu) -> expected z
%   sensor_jacobian     -   h_jac(mu) -> K x 3 Jacobian of the sensor
%   Q                   -   K x K covariance of the measurement noise
%   errThreshold      -   any error > threshold will be slice out from
%
% OUTPUT:
%   mu_next - 3 x 1 mean of the next state
%   sigma_next - 3 x 3 covariance of the next state

% extract mu's dimension
n = size(mu, 1);

% predict
mu_bar = dynamics_model(mu, u);
G = dynamics_jacobian(mu, u);
sigma_bar = G * sigma * G.' + R;

% update

% clip to map range
H = sensor_jacobian(mu_bar);
z = [z_beacon; z_depth];
% slice out measurement that is too different
diff = z - sensor_model(mu_bar);
reasonable = ((~isnan(z)) & (abs(diff) < errThreshold));
diff = diff(reasonable);
H = H(reasonable, :);
Q = Q(reasonable, reasonable);

kGain = sigma_bar * H.' * inv(H * sigma_bar * H.' + Q);
mu_next = mu_bar + kGain * (diff);
sigma_next = (eye(n) - kGain * H) * sigma_bar;

end
