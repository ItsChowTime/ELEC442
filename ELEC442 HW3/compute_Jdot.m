%% Compute the Jacobian_dot

function J_dot = compute_Jdot(q_dot, J, k, coord, O6_dot)

% Calculate Vn(t) (on_dot & wn) for each joint variable except end effector
% Vn = [on_dot; wn] = Jn * q_dot
for i = 1:5
    new_J = J(:,1:i);
    new_q = q_dot(1:i);
    Vn{i,1} = new_J * new_q;
    on_dot{i,1} = Vn{i,1}(1:3);
    w{i,1} = Vn{i,1}(4:6);
end

% Calculate o6 - on and o6_dot - on_dot
for i = 1:5
    o{i,1} = coord{6,1} - coord{i,1};
    o_dot{i,1} = O6_dot - on_dot{i,1};
end

% Construct the Jacobian_dot
k0 = [0;0;1];
w0 = [0;0;0]; % w0 = 0

J1 = [cross(cross(w0,k0),o{1,1}) + cross(k0,O6_dot); cross(w0,k0)];
J2 = [cross(cross(w{1,1},k{1,1}),o{1,1}) + cross(k{1,1},o_dot{1,1}); cross(w{1,1},k{1,1})];
J3 = [cross(cross(w{2,1},k{2,1}),o{2,1}) + cross(k{2,1},o_dot{2,1}); cross(w{2,1},k{2,1})];
J4 = [cross(cross(w{3,1},k{3,1}),o{3,1}) + cross(k{3,1},o_dot{3,1}); cross(w{3,1},k{3,1})];
J5 = [cross(cross(w{4,1},k{4,1}),o{4,1}) + cross(k{4,1},o_dot{4,1}); cross(w{4,1},k{4,1})];
J6 = [cross(cross(w{5,1},k{5,1}),o{5,1}) + cross(k{5,1},o_dot{5,1}); cross(w{5,1},k{5,1})];

J_dot = [J1, J2, J3, J4, J5, J6];