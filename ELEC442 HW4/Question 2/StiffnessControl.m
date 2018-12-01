%% Matlab code for Task Space Stiffness Control

% Initialize constants
C0 = [1, 0; 0, 1];
o0 = [0; 0];
od = [o0 + C0*[1;1]; 0];
Kp = diag([1,1]);      % Kp1 
%Kp = diag([0.2,1]);    % Kp2
%Kp = diag([1,0.2]);    % Kp3
Kv = diag([2,2]);
od_dot = 0;     % Set point = od_dot & omega_d is zero
omega_d = 0;
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
        [x{idx,1},newTheta1(idx), newTheta2(idx)] = robot(x{idx-1,1}, u{idx-1,1}, timestep);
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
    
    % Calculate the forward kinematics to find joint locations
    coord = fwd_kine(q);
    
    % Calculate the error vector
    % Since only end position is given, no need to compute the orientation
    e = od-coord{2,1};  % Desired end effector postion - current position
    e = e(1:2);
    
    % Compute the Jacobian and tranpose_J
    J1 = [-l1*sin(q(1,1)) - l2*sin(q(1,1)+q(2,1));...
         l1*cos(q(1,1)) + l2*cos(q(1,1)+q(2,1))];
    J2 = [-l2*sin(q(1,1)+q(2,1)); l2*cos(q(1,1)+q(2,1))];
    J = [J1, J2];
    tranpose_J = transpose(J);
    
    % Calculate the current translational and rotational velocities
    Vn = J*q_dot;
    
    % Compute the velocity error vector
    e_vel = [od_dot; omega_d] - Vn;
    
    % Compute u
    u{idx,1} = G + tranpose_J*(Kp*e + Kv*e_vel);
    
    % Store the end effector position to plot its trajectory
    o2_x(idx) = coord{2,1}(1);
    o2_y(idx) = coord{2,1}(2);
    
    idx = idx + 1;
end

% Plot the end effector trajectory
figure
subplot(2,1,1)
plot(o2_x,o2_y)
title('End effector (o2) trajectory')
ylabel('j position')
xlabel('i position')

subplot(2,1,2)
plot(newTheta1)
title('same1')
