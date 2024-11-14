function [result,cost] = jacobian_phi_relax(q, pd_dot)
  
    a1 = 0.5; 
    a2 = 0.5;
    k0 = 5;  % Gain 
    
  
    theta1 = q(1);  
    theta2 = q(2);  
    d3 = q(3);      
   
    
    % boundary given in the project 
    theta1_min = -pi/2; 
    theta1_max = pi/2;
    theta2_min = -pi/2; 
    theta2_max = pi/4;
    d3_min = 0.25; 
    d3_max = 1;
    

    % midpoints
    theta1_mid = (theta1_min + theta1_max) / 2;
    theta2_mid = (theta2_min + theta2_max) / 2;
    d3_mid = (d3_min + d3_max) / 2;
    
    % Cost function for plotting, n=3, for q 1 to 3
    w_q = -(1 / (2 * 3)) * (((theta1 - theta1_mid) / (theta1_max - theta1_min))^2 + ...
        ((theta2 - theta2_mid) / (theta2_max - theta2_min))^2 +((d3 - d3_mid) / (d3_max - d3_min))^2);

    cost=w_q;
    
    % Gradient of w(q) (without theta4)
    partial1 = -((theta1 - theta1_mid) / (theta1_max - theta1_min)^2);
    partial2 = -((theta2 - theta2_mid) / (theta2_max - theta2_min)^2);
    partial3 = -((d3 - d3_mid) / (d3_max - d3_min)^2);
    
    
    q_dot_0 = k0 * [partial1; partial2; partial3]; 

    % J while relaxing q4, so 3x3 now
    J = [
        -a1 * sin(theta1) - a2 * sin(theta1 + theta2), -a2 * sin(theta1 + theta2), 0;
         a1 * cos(theta1) + a2 * cos(theta1 + theta2),  a2 * cos(theta1 + theta2), 0;
         0, 0, -1;
    ];

    
    J_pinv = pinv(J);
    
    %implement the equation on slide
    result = J_pinv * pd_dot'  + (eye(3) - J_pinv * J) * q_dot_0;

 
end
