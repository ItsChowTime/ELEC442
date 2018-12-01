function [theta4, negtheta4, theta5, negtheta5, theta6, negtheta6] = calcWrist(C3,k6,j6)
%calcWrist Calculates the angles for a desired orientation for a spherical
%wrist
%   Finds j3 and k3 based on the inputted coordinate frame
    j3 = C3(:,2);
    j3 = j3 / norm(j3);
    
    k3 = C3(:,3);
    k3 = k3 / norm(k3);
    
%   Finds the two positive and negative values for k4
    k4pos = cross(k3, k6)/norm(cross(k3, k6));
    k4neg = -cross(k3,k6)/norm(cross(k3, k6));
    
%   Finds the two possible solutions for the spherical wrist: angle 4,5 and
%   6. The positive theta's represent one solution, the negative theta's
%   the other.

    theta4 = Kahan2(k3, k4pos, j3);
    negtheta4 = Kahan2(k3, k4neg, j3);
    
    theta5 = Kahan2(k4pos, k6, k3); 
    negtheta5 = Kahan2(k4neg, k6, k3);
    
    theta6 = -1 * Kahan2(k6, k4pos, j6);
    negtheta6 = -1 * Kahan2(k6, k4neg, j6);
    
end

