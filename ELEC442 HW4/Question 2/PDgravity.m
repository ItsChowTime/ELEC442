%% Matlab code for PD+gravity Set-Point controller

% Initialize constants
qd = [0; pi/2];
Kp = diag([1,1]);
Kv = diag([2,2]);
qd_dot = 0;     % Set point = qd_dot is zero
m1 = 1;
m2 = 1;
l1 = 1;
l2 = 1;
g = -9.81;

% Initial condition
x{1,1} = [-pi/2; 0; 0; 0];

timestep = 0.05;
idx = 1;

% Controller code
for time = 0:timestep:15    % Loop through for 15s
    
    % Calculate the next robot joint variables and joint velocities
    % Use the 'Robot' code to do so
    if idx > 1 % Ignore if using initial condition
        x{idx,1} = robot(x{idx-1,1}, u{idx-1,1}, timestep);
    end
    
    % Store the state outputs
    q(1,1) = x{idx,1}(1);
    q(2,1) = x{idx,1}(2);
    q_dot(1,1) = x{idx,1}(3);
    q_dot(2,1) = x{idx,1}(4);
    
    % Calculate G(q)
    G1 = (m1+m2)*g*l1*cos(q(1,1)) + m2*g*l2*cos(q(1,1)+q(2,1));
    G2 = m2*g*l2*cos(q(1,1)+q(2,1));
    G = [G1;G2];
    
    % Calculate u
    u{idx,1} = G + Kp*(qd-q) + Kv*(qd_dot-q_dot);
    
    % Store the theta values for plotting
    theta1(idx) = x{idx,1}(1);
    theta2(idx) = x{idx,1}(2);
    
    idx = idx + 1;
end

% Plot the joint angles
figure
subplot(2,1,1)
plot(theta1)
title('JointAngle1')
ylabel('rad/s')

subplot(2,1,2)
plot(theta2)
title('JointAngle2')
xlabel('Time (s)')
ylabel('rad/s')