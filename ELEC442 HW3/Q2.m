%% Matlab code for HW3 Question 2

% Prompt the user to enter od
default1 = {'480 155 455'};
x = inputdlg('Enter the end effector origin (od) separated by spaces: ',...
             'od', [1 50], default1);
od_t = str2num(x{:});
od = transpose(od_t);

% Prompt the user to enter kd
default2 = {'0.769 0.401 0.498'};
y = inputdlg('Enter kd separated by spaces: ', 'kd', [1 50], default2);
kd_t = str2num(y{:});
kd = transpose(kd_t);

% Prompt the user to enter jd
default3 = {'-0.389 -0.325 0.862'};
z = inputdlg('Enter jd separated by spaces: ', 'jd', [1 50], default3);
jd_t = str2num(z{:});
jd = transpose(jd_t);

% Compute the forward kinematics using the home joint variables
q{1,1} = [0; 0; 90; 0; 90; 0];
[k, j, coord] = fwd_kine(q{1,1});
% Record the initial end-effector pose
end_effector_home = coord{6,1};

% Calculate the system angle change
C6_init = calc_C6(q{1,1});
% Compute the desired C6 frame
kd = kd / norm(kd);
jd = jd / norm(jd);
id = cross(jd, kd); % Calculate id
C6_final = [id, jd, kd];
angleChange = EulerAngle(C6_final, C6_init);

delta_t = 0.020; % Time step is 50Hz/20ms
idx = 1;

for time = 0:delta_t:1 % Loop through for 1 sec with 20ms intervals
    
    clf
    % Compute k vectors and joint origin points using fwd kinematics
    [k, j, coord] = fwd_kine(q{idx,1});
    d_trajectory(idx) = norm(coord{6,1}-end_effector_home);
    
    % Compute Vn(t) and Vn_dot(t) for the end effector
    [Vn_dot, Vn] = calcKinematics(od-end_effector_home, time, angleChange);
    
    % Construct the Jacobian
    J = compute_J(k, coord);
    inv_J = inv(J); % Inverse J
    
    % Compute q_dot
    q_dot{idx,1} = inv_J * Vn;
    
    % Construct Jacobian_dot
    On_dot = Vn(1:3);
    J_dot = compute_Jdot(q_dot{idx,1}, J, k, coord, On_dot);
    
    v_trajectory(idx) = On_dot(1);
    On_2dot = Vn_dot(1:3);
    a_trajectory(idx) = On_2dot(1);
    
    % Compute q_2dot
    q_2dot{idx,1} = inv_J * (Vn_dot - (J_dot*q_dot{idx,1}));
    
    % Compute the next joint variables q_i+1
    q{idx+1,1} = q{idx,1} + (q_dot{idx,1}*delta_t) + (0.5*q_2dot{idx,1}*(delta_t)^2);
        
    % Increment idx
    idx = idx + 1;
    
    % Plot the 3D animations of all joint variables
    hold on
    for t = 1:6
        plotPoint(coord{t,1}, j{t,1}, k{t,1});
        plot3([coord{1}(1), coord{2}(1)], [coord{1}(2), coord{2}(2)], [coord{1}(3), coord{2}(3)]);
        plot3([coord{2}(1), coord{3}(1)], [coord{2}(2), coord{3}(2)], [coord{2}(3), coord{3}(3)]);
        plot3([coord{3}(1), coord{4}(1)], [coord{3}(2), coord{4}(2)], [coord{3}(3), coord{4}(3)]);
face        plot3([coord{4}(1), coord{5}(1)], [coord{4}(2), coord{5}(2)], [coord{4}(3), coord{5}(3)]);
        plot3([coord{5}(1), coord{6}(1)], [coord{5}(2), coord{6}(2)], [coord{5}(3), coord{6}(3)]);
    end
        
   
    hold off
    pause(0.05);
end

% Plot the trajectory graphs
figure;
subplot(3,1,1);
d_max = max(d_trajectory);
plot(0:delta_t:1, d_trajectory);
axis([0, 1, 0, d_max]);
title('Displacement Trajectory');
ylabel('mm');

subplot(3,1,2);
v_max = max(v_trajectory);
plot(0:delta_t:1, v_trajectory);
axis([0, 1, 0, v_max]);
title('Velocity Trajectory');
ylabel('mm/s');

subplot(3,1,3);
a_max = max(a_trajectory);
a_min = min(a_trajectory);
plot(0:delta_t:1, a_trajectory);
axis([0, 1, a_min-0.1, a_max+0.1]);
title('Acceleration Trajectory');
xlabel('Time (s)');
ylabel('mm/s^2');