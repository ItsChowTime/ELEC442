
%Prompt user to enter initial joint variable conditions
default1 = {'0 0 0 0'}; % 80 deg test
x = inputdlg('Enter the initial joint variable conditions at t=0 (xd) separated by spaces: ',...
             'xd', [1 50], default1);
xd_t = str2num(x{:});
xd = transpose(xd_t);

q1 = xd(1);
q2 = xd(2);
q1dot = xd(3);
q2dot = xd(4);

%Prompt user to enter constant torque values u1 and u2
default2 = {'0 0'};
u = inputdlg('Now enter the torque values for the robot, separated by spaces: ',...
             'ud', [1 50], default2);
ud_t = str2num(u{:});
ud = transpose(ud_t);

u1=ud(1);
u2=ud(2);

%Sets timestep for simulation
timestep = 0.05;
count = 1;
q1next(1) = q1;
q2next(1) = q2;
q1dotnext(1) = q1dot;
q2dotnext(1) = q2dot;

torqueConstant1 = -0.05;
torqueConstant2 = -0.05;

for time = 0:timestep:30-timestep
    
    %Calculates the joint accelerations given the current joint variables,
    %velocities and torques
    [q1_2dot(count), q2_2dot(count)] = torque_2_accel(q1, q2, q1dot, q2dot, u2, u1);
    
    %Calculates the new joint variable given the acceleration
    q1next(count) = q1 + q1dot*timestep + 1/2*q1_2dot(count)*timestep^2;
    q2next(count) = q2 + q2dot*timestep + 1/2*q2_2dot(count)*timestep^2;
    
    %Calculates the new joint velocities by dividing the difference in
    %angle change by the timestep
    q1dotnext(count) = (q1next(count) - q1)/timestep;
    q2dotnext(count) = (q2next(count) - q2)/timestep;
    
    q1 = q1next(count);
    q2 = q2next(count);
    q1dot = q1dotnext(count);
    q2dot = q2dotnext(count);
    
    x_n = [q1; q2; q1dot; q2dot];
    
    %Determines the new torque for Part C of Question 1;
    %u1 = q1dot*torqueConstant1;
    %u2 = q2dot*torqueConstant2;
    
    
    %Increments time step
    count = count+1;

end

%Plots Joint angles
figure
subplot(4,1,1)
plot(q1next)
title('JointAngle1')

subplot(4,1,2)
plot(q2next)
title('JointAngle2')

subplot(4,1,3)
plot(q1dotnext)
title('velocity1')

subplot(4,1,4)
plot(q2dotnext)
title('velocity2')