%% Compute C6 - the sixth/final frame
% Assume input angles are in degrees
function C6 = calc_C6(angles)

% Calculate the base frame C0
i0 = [1; 0; 0];
j0 = [0; 1; 0];
k0 = [0; 0; 1];
C0 = [i0, j0, k0];

% Compute the remaining frames
C1 = C0*rotz(angles(1,1))*rotx(-90);
C2 = C1*rotz(angles(2,1))*rotx(180);
C3 = C2*rotz(angles(3,1)+90)*rotx(90);
C4 = C3*rotz(angles(4,1))*rotx(90);
C5 = C4*rotz(angles(5,1))*rotx(-90);
C6 = C5*rotz(angles(6,1));