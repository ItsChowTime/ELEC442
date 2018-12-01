    %% Compute C3 - the third frame
% Assume input angles are in degrees
function C3 = calc_C3(theta1, theta2, theta3)

% Calculate the base frame C0
i0 = [1; 0; 0];
j0 = [0; 1; 0];
k0 = [0; 0; 1];
C0 = [i0, j0, k0];

% Compute the remaining frames
C1 = C0*rotz(theta1)*rotx(-90);
C2 = C1*rotz(theta2)*rotx(180);
C3 = C2*rotz(theta3+90)*rotx(90);