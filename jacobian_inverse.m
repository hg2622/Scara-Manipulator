function result = jacobian_inverse(q, xd_dot)
    
    a1 = 0.5; 
    a2 = 0.5;

    % extract joint velocities
    pd_dot = xd_dot(1:3);   
    theta_dot = xd_dot(4); 
    
    % extract joints from q
    theta1 = q(1);  
    theta2 = q(2);  
        

   

    % the derived jacobian from calculations
    J = [
        -a1 * sin(theta1) - a2 * sin(theta1 + theta2), -a2 * sin(theta1 + theta2), 0, 0;
         a1 * cos(theta1) + a2 * cos(theta1 + theta2),  a2 * cos(theta1 + theta2), 0, 0;
         0, 0, -1, 0;
         1, 1 0, 1
    ];


    % matlab's built in inverse function
    J_inverse = inv(J);

   
    result = J_inverse * [pd_dot; theta_dot];
end
