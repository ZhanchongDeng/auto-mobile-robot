function [mu_next, sigma_next] = EKF(mu, sigma, u, z, dynamics_model, ...
    dynamics_jacobian, R, sensor_model, sensor_jacobian, Q, errThreshold)
% EKF: One step EKF(Extended Kalman Filter), predict and update
%
% INPUT:
%   mu                  -   3 x 1 mean of the state [x; y; theta]
%   sigma               -   3 x 3 covariance of the state
%   u                   -   control input [d; phi]
%   z                   -   measurement (could be GPS pose [x; y; theta] or depth K x 1)
%   dynamics_model      -   g(mu, u) -> next mu
%   dynamics_jacobian   -   g_jac(mu, u) -> 3 x 3 Jacobian of the dynamics
%   R                   -   3 x 3 covariance of the control noise
%   sensor_model        -   h(mu) -> expected z
%   sensor_jacobian     -   h_jac(mu) -> K x 3 Jacobian of the sensor
%   Q                   -   K x K covariance of the measurement noise
%   errThreshold      -   any error > threshold will be slice out from
%   update
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

% slice out measurement that is too different
diff = z - sensor_model(mu_bar);
reasonable = (abs(diff) < errThreshold);
diff = diff(reasonable);
H = H(reasonable, :);
Q = Q(reasonable, reasonable);

kGain = sigma_bar * H.' * inv(H * sigma_bar * H.' + Q);
mu_next = mu_bar + kGain * (diff);
sigma_next = (eye(n) - kGain * H) * sigma_bar;

end
