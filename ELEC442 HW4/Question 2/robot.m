function [x_n,newTheta1, newTheta2] = robot(theta, torque, timestep)
% Calculates the joint origin accelerations given the positions, and torque
% This 'Robot' code is used in conjuction with the 'Controller' codes for
% questions 2 and 3
   
   theta = rad2deg(theta);
   theta1 = theta(1);
   theta2 = theta(2);
   theta1dot = theta(3);
   theta2dot = theta(4);
   
   torque1 = torque(1);
   torque2 = torque(2);
   syms a b
   
   %Follows Newton-Euler approach to find torque, in this case torque is
   %known, so theta2 and theta1 double dot is found instead   
 
   rightSide2 = torque2 - (theta1dot^2*sind(theta2)) - (-9.81*cosd(theta1+theta2)); 
   rightSide1 = torque1 - (theta1dot^2*sind(theta2)) - (-9.81*cosd(theta1+theta2))...
   + ((theta1dot + theta2dot)^2*sind(theta2)) - (-9.81*cosd(theta1)) ...
   - (-9.81*cosd(theta1)); 

   leftSide2 = (a + b) + a*cosd(theta2);
   leftSide1 = (1 + cosd(theta2))*a + b + a + ((a+b)*cosd(theta2)) + a;
   
   
   %Creates equations to solve linear system of equations
   eqn1 = leftSide1 == rightSide1;
   eqn2 = leftSide2 == rightSide2;
   
   [X,Y] = equationsToMatrix([eqn1, eqn2], [a,b]);
   
   %Solves for joint accelerations
   result = linsolve(X,Y);
   
   theta1_2dot = eval(result(1));
   theta2_2dot = eval(result(2));
   
   %Calculates the new joint variable given current joint variables,
   %velocities and accelerations
   newTheta1 =(theta1 + theta1dot*timestep + 1/2*theta1_2dot*timestep^2);
   newTheta2 =(theta2 + theta2dot*timestep + 1/2*theta2_2dot*timestep^2);
   
   newTheta1dot = (newTheta1 - theta1)/timestep;
   newTheta2dot = (newTheta2 - theta2)/timestep;
   
   %Puts variables into vectors
   x_n = deg2rad([newTheta1; newTheta2; newTheta1dot; newTheta2dot]); 
   
end

