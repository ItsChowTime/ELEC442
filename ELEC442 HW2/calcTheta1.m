function [theta1, theta2] = calcTheta1(WristCenter)
%calcTheta1
%   Calculates theta 1 of a PUMA560 for assgt. 2
%   Utilizes function Kahan2, which calculates the angle between two
%   vectors about a rotation vector.

    Plane = 149.1;                  %Predefined plane Y = -149.1, found by observing that when theta1 is equal to 0
                                    %the manipulator can only move about
                                    %this plane
                                    
    Distance = norm(WristCenter);   %Distance from wrist center to origin
    
    AxisOfRotation = [0,0,1];       %Theta 1 rotates about the k0 axis
    
    pointZ = WristCenter(3);        %When only theta 1 is being changed, the z-components
                                    %of both vectors will be equal
    
    %Finds the first intersection x-coordinate of the wrist center to the plane
    pointX1 = sqrt(Distance^2 - abs(Plane^2) - pointZ^2);  
    %Finds the second intersection x-coordinate of the wrist center to the plane
    pointX2 = -sqrt(Distance^2 - abs(Plane^2) - pointZ^2);
    
    %Finds exact coordinates of intersection for both solutions
    newWristCenter1 = [pointX1, Plane, pointZ];
    newWristCenter2 = [pointX2, Plane, pointZ];
    %Calculates the two theta solutions using Kaden-Pahan sub-problem 2
    theta1 = Kahan2(AxisOfRotation, newWristCenter1, WristCenter);
    theta2 = Kahan2(AxisOfRotation, newWristCenter2, WristCenter);
    
end

