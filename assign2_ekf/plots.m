a = 4.5 * 9.8; # acceleration (m/s^2)
dt = 0.5;      # delta-time (s)
t = 0:dt:4;    # time ticks (s)

# closed form solution
closed = 0.5 * a * t.^2;

# euler integration
euler = [];
v = 0; # velocity or speed (m/s)
s = 0; # distance travelled (m)
for i = t
  euler = [euler s];
  s = s + v*dt;
  v = v + a*dt;
end

# taylor expansion
taylor = [];
v = 0;
s = 0;
for i = t
  taylor = [taylor s];
  s = s + v*dt + a/2*dt^2;
  v = v + a*dt;
end

figure;
hold on;
plot(t, closed, 'LineWidth', 2);
plot(t, euler, 'LineWidth', 2);
plot(t, taylor, 'LineWidth', 2);
l=legend('Closed', 'Euler', 'Taylor', 'Location', 'North');
set(l, 'FontSize', 16);
set(gca, 'FontSize', 16);