%% Compute theta2 and theta3 in a 2D inv kinematics problem
% Given the wrist center (o4)
function [elbow_down_theta2, elbow_down_theta3, ...
    elbow_up_theta2, elbow_up_theta3] = two_D_inv(o4)

% Since this is a 2D probLem, onLy need the i0 and k0 components of the
% wrist center
a = o4(1); % i0 of wrist center
b = o4(3); % k0 of wrist center
L_squared = a^2 + b^2; % L^2 = a^2 + b^2

L1 = 430.0; % distance between joint 1 and 2
L2 = sqrt(435^2 + 20.3^2); % pythagoras of length and height offset between joints 2 & 4

% solve for theta3 (2 solutions, positive and negative angle)
% formula: L^2 = L1^2 + L2^2 + 2*L1*L2*cos(theta3)
theta3 = acosd((L_squared - L1^2 - L2^2) / (2*L1*L2));
negtheta3 = -theta3;
% Check to see if both theta3 answers are within the bounds
% Returns the angle if it is, returns -1 if it isn't
%elbow_down_theta3 = CheckBounds(theta3, -135, 135);
%elbow_up_theta3 = CheckBounds(negtheta3, -135, 135);
elbow_down_theta3 = theta3;
elbow_up_theta3 = negtheta3;

% solve for theta2
% formula: L2^2 = L1^2 + L^2 - 2*L1*L*cos(theta2)
alpha = atand(b/a);
beta = acosd((L2^2 - L1^2 - L_squared)/(-2*L1*sqrt(L_squared)));
theta2 = alpha - beta;
negtheta2 = alpha + beta;
% Check to see if both theta2 answers are within the bounds
% Returns the angle if it is, returns -1 if it isn't
%elbow_down_theta2 = CheckBounds(theta2, -225, 45);
%elbow_up_theta2 = CheckBounds(negtheta2, -225, 45);
elbow_down_theta2 = -theta2;
elbow_up_theta2 = -negtheta2;