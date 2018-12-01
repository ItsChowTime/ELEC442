%% Kahan P3 code
% Given s, t, u, v, find theta and phi
% Produces 2 solns for both theta and phi
function[theta1, theta2, phi1, phi2] = Kahan3(s,t,u,v)

% Normalize the vectors
s = s / norm(s);
t = t / norm(t);
u = u / norm(u);
v = v / norm(v);

% Tranpose the vectors
st = transpose(s);
tt = transpose(t);
ut = transpose(u);
vt = transpose(v);

% Dot product vectors together
ut_s = dot(ut,s); % ut.s
vt_t = dot(vt,t); % vt.t
st_t = dot(st,t); % st.t
tt_s = dot(tt,s); % tt.s

% Solve for alpha and beta
matrix = [1, st_t; tt_s, 1]; % Create the 2x2 matrix
matrix_i = inv(matrix); % Inverse the 2x2 matrix
vector = matrix_i*[ut_s; vt_t]; % Multiply the inv_matrix by ut.s & vt.t
alpha = vector(1,1);
beta = vector(2,1);

% Compute z
z = alpha*s + beta*t;
mag_z = norm(z);

% Check to see if mag_z > 1
if mag_z > 1
    % no solutions
    theta1 = [];
    theta2 = [];
    phi1 = [];
    phi2 = [];
else
    % Compute cross product of s & t
    sxt = cross(s,t);
    mag_sxt = norm(sxt);
    norm_sxt = sxt/mag_sxt;
    
    % Compute w, should have 2 solns
    term = sqrt(1-(mag_z^2))*norm_sxt;
    w = [z+term; z-term]; % 2 solns
    
    % Apply Kahan2 both w solns to find theta1, theta2, phi1, phi2
    theta1 = Kahan2(s,w(1,:),u);
    theta2 = Kahan2(s,w(2,:),u);
    phi1 = Kahan2(s,w(1,:),u);
    phi2 = Kahan2(s,w(2,:),u);
end