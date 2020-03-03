m_bottle = 0.013; % kg
d_throw  = 5;   % meters
g = -9.81
V_max = 2.5;
theta = pi/4;

X_0 = [0 0.2]';
V_0 = [V_max*cos(theta) V_max*sin(theta)]';
A_0 = [0 g/2]';
X = [];

for t = linspace(0, d_throw/V_0(1))
    X = [X A_0*t^2+V_0*t+X_0];
end

figure(1)

hold off
plot(X(1,:), X(2,:))
hold on
plot([0.15 0.15], [0 0.25])
ylim([0 1])

%% Kinematics

L1 = 0.25;
M1 = 0.5;

L2 = 0.37;
M2 = 0.4;

L3 = 0.37;
M3 = 0.3;

L4 = 0.1;
M4 = 0.3;

