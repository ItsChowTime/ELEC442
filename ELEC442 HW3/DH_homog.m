% Accepts four DH params and outputs the
% homogenous transformation matrix of the 2 links
function homog = DH_homog(theta_deg, d, a, alpha_deg)

% Change the input angles from degrees to radians
theta = theta_deg * pi / 180;
alpha = alpha_deg * pi / 180;

% Initialize matrix variables
homog = eye(4);

% Calculate the cos and sin of the angles
ct = cos(theta);
st = sin(theta);
ca = cos(alpha);
sa = sin(alpha);

% Construct the homogenous transformation matrix
homog = [ct -st*ca st*sa a*ct ; ...
    st ct*ca -ct*sa a*st ; ...
    0 sa ca d ; ...
    0 0 0 1];