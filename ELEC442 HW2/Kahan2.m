function [ theta ] = Kahan2( s, w, u )
%Kahan2 Summary of this function goes here
%   Calculates the angle between two vectors given a rotation axis

    %Normalize all vectors
    normalS = s / norm(s);
    normalW = w / norm(w);
    normalU = u / norm(u);

    %Ensuring rotation locus intersect
    if(dot(transpose(normalS),normalU) == dot(transpose(normalS), normalW))
        
        %Follows Kaden-Pahan Sub-Problem 2 as discussed in class
        theta1 = 2 * atan(norm(cross(normalS,normalU - normalW))/norm(cross(normalS,(normalU + normalW))));
        signTheta = sign(dot(transpose(normalW),cross(normalS, normalU - normalW)));   %Checking for sign of theta
        theta = -rad2deg(theta1*signTheta);         %Converts radians to degrees
    else
        fprintf('Solution does not exist');
        theta = -1;
    end
end

