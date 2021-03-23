clc;
clear all;

% input data (camera angle (rad) & rocket acceleration (m/s^2))
alphas = [0.040,0.156,0.329,0.533,0.769,0.952,1.085,1.189,1.270,1.317,1.343,1.410,1.434,1.439,1.460,1.459];
accels = [4.11,4.76,3.81,4.08,2.23,2.06,4.63,3.64,3.88,1.56,4.14,5.20,3.54,3.64,3.84,2.46];

ground_truth = 0.5 * (0.4 * 9.8) * (0:30).^2;

% state
s = 0; % altitude (m)
v = 0; % velocity (m/s)
S = [
  0, 0;
  0, 0
];

kalman = [];
for i = 1:16
  kalman = [kalman, s];
  
  % inputs
  a = accels(i); % rocket acceleration (m/s^2)
  z = alphas(i); % camera angle (rad)
  d = 200;       % horizontal distance rocket<->camera (m)
  dt = 2;        % timestep (s)
  
  % function g(mu)
  % TODO: Improve using Taylor expansion:
  s = s + v * dt; % s = g1(mu)
  v = v + a * dt; % v = g2(mu)
  
  G = [
    deriv. of g1 wrt. s, deriv. of g1 wrt. v;
    deriv. of g2 wrt. s, deriv. of g2 wrt. v
  ];
  
  R = [
    est. error in s, 0;
    0, est. error in v
  ].^2;
  
  S = G*S*G' + R;
  
  % function h(mu)
  hmu = ...;
  
  H = [deriv. of h wrt. s, deriv. of h wrt. v];
  
  Q = ...^2;
  
  K = S*H'*inv(H*S*H' + Q);
  mu = [s; v] + K*(z - hmu);
  S = (eye(2) - K*H)*S;
  
  s = mu(1); v = mu(2);
end

figure;
hold on;
plot(0:30, ground_truth, 'LineWidth', 2, 'Color', 'g');
plot(0:2:30, kalman, 'LineWidth', 2, 'Color', 'r');
l=legend('ground truth', 'estimate', 'Location', 'North');
set(l, 'FontSize', 16);
set(gca, 'FontSize', 16);

% MSE
mean((ground_truth(1:2:end) - kalman).^2)