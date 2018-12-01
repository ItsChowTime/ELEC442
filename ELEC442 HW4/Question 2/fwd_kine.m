%% Compute the forward kinematics
% Input: Joint variables in deg
% Output: k & j vectors and joint origin points for every joint variable
function [coord] = fwd_kine(angles)

% Calculate the homogenous transformation matrices for every link
homog{1,1} = DH_homog(angles(1,1), 0, 1, 0);
homog{2,1} = DH_homog(angles(2,1), 0, 1, 0);



% Calculate the T matrices wrt the base coords
T{1,1} = homog{1,1};
T{2,1} = homog{1,1}*homog{2,1};


% Multiply each base T matrix by [0;0;1;0] to get the k & j vectors
for i = 1:2
    k{i,1} = T{i,1} * [0;0;1;0];
    j{i,1} = T{i,1} * [0;1;0;0];
    % Take the first 3 indexes of the vectors
    k{i,1} = k{i,1}(1:3);
    j{i,1} = j{i,1}(1:3);
end

% Multiply each base T matrix by [0;0;0;1] to get the origin point
for i = 1:2
    coord{i,1} = T{i,1} * [0;0;0;1];
    % Take the first 3 indexes of the coord vector
    coord{i,1} = coord{i,1}(1:3);
end