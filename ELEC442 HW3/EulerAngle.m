%% Compute the Euler angles for the entire system rotation
% Math behind this can be found in Q1
function [angleChange] = EulerAngle(frame_f,frame_i)

% Calculate the rotation matrix
R = frame_f / frame_i;

% Compute theta_i, theta_j, theta_k rotations
angleChange(1) = radtodeg(atan2(R(3,2), R(3,3)));
angleChange(2) = radtodeg(atan2(-R(3,1), sqrt(R(3,2)^2 + R(3,3)^2)));
angleChange(3) = radtodeg(atan2(R(2,1), R(1,1)));
    
end

