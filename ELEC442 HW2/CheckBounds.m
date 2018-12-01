function [Anglebound] = CheckBounds(angle, lowBound, hiBound)
%%CheckBounds Function to check if an angle is within a determined bound

    
if((angle >= lowBound)&&(angle <= hiBound))
    Anglebound = angle; %Returns the angle if the angle is within the bound
else
    Anglebound = -1; %Returns -1 if the angle is not within the bound NOTE: may need to be changed from -1 
end
    
end

