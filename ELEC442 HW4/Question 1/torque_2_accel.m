function [theta1_2dot, theta2_2dot] = torque_2_accel(theta1, theta2, theta1dot, theta2dot, torque1, torque2)
%torque_2_accel Calculates the joint origin accelerations given the
%positions, and torque
   
   %length1 = 1;
   %length2 = 1;
   %mass2 = 1;
   %mass1 = 1;
   U = [torque1 ; torque2];
   Qdot = [theta1dot ; theta2dot];
   D = [ 3 + 2*cosd(theta2), 1+cosd(theta2) ; 1+cosd(theta2), 1];
   C = [(-2*sin(theta2)*theta1dot*theta2dot) - (sin(theta2)*theta2dot^2) ; (sin(theta2)*theta1dot^2)];
   G = [2*9.81*cos(theta1) + 9.81*cos(theta1+theta2) ; 9.81*cos(theta1+theta2)];
   syms a b
   
   q2dot =  inv(D)*(U - C - G);
   %Follows Newton-Euler approach to find torque, in this case torque is
   %known, so theta2 and theta1 double dot is found instead   
 
   %rightSide2 = torque2 - (theta1dot^2*sind(theta2)) - (9.81*cosd(theta1+theta2)); 
   %rightSide1 = torque1 - (theta1dot^2*sind(theta2)) - (9.81*cosd(theta1+theta2))...
   %+ ((theta1dot + theta2dot)^2*sind(theta2)) - (9.81*cosd(theta1)) ...
   %- (9.81*cosd(theta1)); 

   %leftSide2 = (a + b) + a*cosd(theta2);
   %leftSide1 = (1 + cosd(theta2))*a + b + a + ((a+b)*cosd(theta2))+a;
   
   %Creates equations to solve linear system of equations
   %eqn1 = leftSide1 == rightSide1;
   %eqn2 = leftSide2 == rightSide2;
   
   %[X,Y] = equationsToMatrix([eqn1, eqn2], [a,b]);
   
   %Solves for joint accelerations
   %result = linsolve(X,Y);
   
   theta1_2dot = q2dot(1);
   theta2_2dot = q2dot(2);
   

end

