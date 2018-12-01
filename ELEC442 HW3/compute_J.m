%% Compute the Jacobian

function J = compute_J(k, coord)

% Calculate o_6 - o_n
for i = 1:5
    o{i,1} = coord{6,1} - coord{i,1};
end

% Construct the Jacobian
k0 = [0;0;1];
J = [cross(k0,o{1,1}), cross(k{1,1},o{1,1}), cross(k{2,1},o{2,1}), ...
    cross(k{3,1},o{3,1}), cross(k{4,1},o{4,1}), cross(k{5,1},o{5,1}); ...
    k0, k{1,1}, k{2,1}, k{3,1}, k{4,1}, k{5,1}];