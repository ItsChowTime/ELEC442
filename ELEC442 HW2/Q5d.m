%% Matlab code for Question 5d

% Prompt the user to enter in the joint angles in degrees
prompt = {'Enter joint angle 1 (deg):', 'Enter joint angle 2 (deg):', ...
    'Enter joint angle 3 (deg):', 'Enter joint angle 4 (deg):', ...
    'Enter joint angle 5 (deg):', 'Enter joint angle 6 (deg):'};
dlg_title = 'Input';
num_lines = 1;
defaultans = {'45', '-45', '45', '0', '-30', '90'};
answer = inputdlg(prompt,dlg_title,num_lines,defaultans);

% Convert the user inputs into numeric values and store them
angles = zeros(6,1);
for i = 1:6
    angles(i,1) = str2double(answer{i,1});
end

% Calculate the homogenous transformation matrices for every link
homog{1,1} = DH_homog(angles(1,1), 0, 0, -90);
homog{2,1} = DH_homog(angles(2,1), 0, 430.0, 180);
homog{3,1} = DH_homog(angles(3,1)+90, -149.1, 20.3, 90);
homog{4,1} = DH_homog(angles(4,1), 435., 0, 90);
homog{5,1} = DH_homog(angles(5,1), 0, 0, -90);
homog{6,1} = DH_homog(angles(6,1), 60.00, 0, 0);

% Calculate the T matrices wrt the base coords
T{1,1} = homog{1,1};
T{2,1} = homog{1,1}*homog{2,1};
T{3,1} = homog{1,1}*homog{2,1}*homog{3,1};
T{4,1} = homog{1,1}*homog{2,1}*homog{3,1}*homog{4,1};
T{5,1} = homog{1,1}*homog{2,1}*homog{3,1}*homog{4,1}*homog{5,1};
T{6,1} = homog{1,1}*homog{2,1}*homog{3,1}*homog{4,1}*homog{5,1}*homog{6,1};
homogT = T{6,1}

% Multiply each base T matrix by [0;0;1;0] to get the k vectors
for i = 1:6
    k{i,1} = T{i,1} * [0;0;1;0];
end

% Multiply each base T matrix by [0;0;0;1] to get the origin point
for i = 1:6
    coord{i,1} = T{i,1} * [0;0;0;1];
end

% Calculate o_6 - o_n and keep the first 3 indexes
% Take the first 3 indexes of the k vector
for i = 1:6
    temp = coord{6,1}-coord{i,1};
    o{i,1} = temp(1:3);
    k{i,1} = k{i,1}(1:3);
end

% Construct the Jacobian
k0 = [0;0;1];
J = [cross(k0,o{1,1}), cross(k{1,1},o{1,1}), cross(k{2,1},o{2,1}), ...
    cross(k{3,1},o{3,1}), cross(k{4,1},o{4,1}), cross(k{5,1},o{5,1}); ...
    k0, k{1,1}, k{2,1}, k{3,1}, k{4,1}, k{5,1}]

% Plot the results
 plot3(coord{1,1},coord{2,1},coord{3,1},coord{4,1},coord{5,1},coord{6,1});