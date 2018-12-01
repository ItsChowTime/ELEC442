%% Matlab code for HW2

% TODO: interface with all functions, display results
% Prompt the user to enter od
default1 = {'317 506 673'};
x = inputdlg('Enter the end effector origin (od) separated by spaces: ',...
             'od', [1 50], default1);
od_t = str2num(x{:});
od = transpose(od_t);

% Prompt the user to enter kd
default2 = {'0.769 0.401 0.498'};
y = inputdlg('Enter kd separated by spaces: ', 'kd', [1 50], default2);
kd_t = str2num(y{:});
kd = transpose(kd_t);

% Prompt the user to enter jd
default3 = {'-0.389 -0.325 0.862'};
z = inputdlg('Enter jd separated by spaces: ', 'jd', [1 50], default3);
jd_t = str2num(z{:});
jd = transpose(jd_t);

id = cross(jd, kd); % Calculate id
Cd = [id, jd, kd]; % Compute the end effector frame

% Calculate the origin coords of the wrist center
o4 = od - 60.0*kd;

% Find theta1 (2 solutions)
[theta1(1), theta1(2)] = calcTheta1(transpose(o4));

% Find theta2 and theta3 (2 solutions)
[theta2(1), theta3(1), theta2(2), theta3(2)] = two_D_inv(o4);

% Calculate C3, the third frame (2 solutions for C3 due to 2 theta1 angles)
C3{1} = calc_C3(theta1(1), theta2(1), theta3(1));
C3{2} = calc_C3(theta1(2), theta2(1), theta3(1));
C3{3} = calc_C3(theta1(1), theta2(2), theta3(2));
C3{4} = calc_C3(theta1(2), theta2(2), theta3(2));

% Find theta4, theta5, theta6 (2 solutions)
[theta4(1), theta4(2), theta5(1), theta5(2), theta6(1), theta6(2)] = calcWrist(C3{1},kd,jd);
[theta4(3), theta4(4), theta5(3), theta5(4), theta6(3), theta6(4)] = calcWrist(C3{2},kd,jd);
[theta4(5), theta4(6), theta5(5), theta5(6), theta6(5), theta6(6)] = calcWrist(C3{3},kd,jd);
[theta4(7), theta4(8), theta5(7), theta5(8), theta6(7), theta6(8)] = calcWrist(C3{4},kd,jd);

% Check to see if the angles are within the boundaries
% CheckBounds.m checks the boundaries of the angle, returns -1 if outside
% If one angle in a solution is outside the range, discard the entire set
idx = 1;
for i = 1:length(theta1)
    theta1(i) = CheckBounds(theta1(i), -160, 160);
    if theta1(i) ~= -1
        for x = 1:length(theta2)
            theta2(x) = CheckBounds(theta2(x), -225, 45);
            theta3(x) = CheckBounds(theta3(x), -135, 135);
            if (theta2(x)~=-1) && (theta3(x)~=-1)
                for y = 1:length(theta4)
                    theta4(y) = CheckBounds(theta4(y), -110, 170);
                    theta5(y) = CheckBounds(theta5(y), -100, 100);
                    theta6(y) = CheckBounds(theta6(y), -266, 266);
                    if (theta4(y)~=-1) && (theta5(y)~=-1) && (theta6(y)~=-1)
                        valid{idx} = [theta1(i), theta2(x), theta3(x),...
                            theta4(y), theta5(y), theta6(y)];
                        idx = idx + 1;
                    end
                end
            end
        end
    end
end

% Print the valid solutions
for z = 1:length(valid)
    disp(valid{z});
end