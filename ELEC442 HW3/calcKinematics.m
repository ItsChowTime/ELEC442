function [VnDot, Vn] = calcKinematics(origin, time, angleChange)
%
%   Purpose: Calculates the translational or angular acceleration, velocity
%   and position of the end effector pose 
%   Input: change in position or angle for desired end effector pose
%   Output: required accleration, velocity, and position for time step to
%   satisfy desired kinematic profiles
%   Mathematic Principles: uses basic kinematic and angular kinematic
%   equations
%
    
    %Calculates base values for acceleration, velocity, displacement in all
    %3 axis directions
    ax = 4*abs(origin(1)); 
    vx = 4*abs(origin(1))*time;
    dx = 2*abs(origin(1))*time^2;
    
    ay = 4*abs(origin(2));
    vy = 4*abs(origin(2))*time;
    dy = 2*abs(origin(2))*time^2;
    
    az = 4*abs(origin(3));
    vz = 4*abs(origin(3))*time;
    dz = 2*abs(origin(3))*time^2;
    
    %Calculates base values for angular acceration and velocity, and
    %relative angle
    wdot_i = 4*abs(angleChange(1));
    w_i = 4*abs(angleChange(1))*time;
    theta_i = 2*abs(angleChange(1))*time^2;
    
    wdot_j = 4*abs(angleChange(2));
    w_j = 4*abs(angleChange(2))*time;
    theta_j = 2*abs(angleChange(2))*time^2;  
    
    wdot_k = 4*abs(angleChange(3));
    w_k = 4*abs(angleChange(3))*time;
    theta_k = 2*abs(angleChange(3))*time^2;
    
    %Performs time check to determine if the end effector is accelerating
    %or decelerating 
    
    %Accelerates for the first half of the motion
    if time < 0.5
       acceleration(1) = ax;
       acceleration(2) = ay;
       acceleration(3) = az;
       
       velocity(1) = vx;
       velocity(2) = vy;
       velocity(3) = vz;
       
       position(1) = dx;
       position(2) = dy;
       position(3) = dz;
       
       angA(1) = wdot_i;
       angA(2) = wdot_j;
       angA(3) = wdot_k;
       
       angV(1) = w_i;
       angV(2) = w_j;
       angV(3) = w_k;
       
       angD(1) = theta_i;
       angD(2) = theta_j;
       angD(3) = theta_k;
       
    %Acceleration will be 0 at t = 0.5
    elseif time == 0.5
        acceleration = [0,0,0];
        velocity(1) = vx;
        velocity(2) = vy;
        velocity(3) = vz;
        
        position(1) = dx;
        position(2) = dy;
        position(3) = dz;

       angA = [0,0,0];
       
       angV(1) = w_i;
       angV(2) = w_j;
       angV(3) = w_k;
       
       angD(1) = theta_i;
       angD(2) = theta_j;
       angD(3) = theta_k;
       
    %Decelerates for second half of motion
    else 
        acceleration(1) = -ax;
        acceleration(2) = -ay;
        acceleration(3) = -az;
        
        velocity(1) = -vx + ax;
        velocity(2) = -vy + ay;
        velocity(3) = -vz + az;
        
        position(1) = -dx + vx - ax/4;
        position(2) = -dy + vy - ay/4;
        position(3) = -dz + vz - az/4;
        
       angA(1) = -wdot_i;
       angA(2) = -wdot_j;
       angA(3) = -wdot_k;
       
       angV(1) = -w_i + wdot_i;
       angV(2) = -w_j + wdot_j;
       angV(3) = -w_k + wdot_k;
       
       angD(1) = -theta_i + w_i - wdot_i/4;
       angD(2) = -theta_j + w_j - wdot_j/4;
       angD(3) = -theta_k + w_k - wdot_k/4;
    end
    
    Vn = [velocity'; angV'];
    VnDot = [acceleration'; angA'];
end

