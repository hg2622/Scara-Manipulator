function result = jacobian_inverse(q, xe_dot_dot)
    disp(xe_dot_dot)
    a1 = 0.5; 
    a2 = 0.5;
    
    % extract joints from q
    theta1 = q(1);  
    theta2 = q(2);  
        
    % the derived jacobian from calculations
  J = [
        -a1 * sin(theta1) - a2 * sin(theta1 + theta2), -a2 * sin(theta1 + theta2), 0, 0;
         a1 * cos(theta1) + a2 * cos(theta1 + theta2),  a2 * cos(theta1 + theta2), 0, 0;
         0, 0, -1, 0; % z component
         1, 1, 0, 1
    ];

    J

    result = inv(J) * xe_dot_dot;
    
    
end

