function xe_dot = jacobian(q,q_dot)

    a1 = 0.5; 
    a2 = 0.5;

    % extract joints from q
    theta1 = q(1);  
    theta2 = q(2);  

  J = [
        -a1 * sin(theta1) - a2 * sin(theta1 + theta2), -a2 * sin(theta1 + theta2), 0, 0;
         a1 * cos(theta1) + a2 * cos(theta1 + theta2),  a2 * cos(theta1 + theta2), 0, 0;
         0, 0, -1, 0;
         1, 1 0, 1
    ];


  xe_dot=J*q_dot;


end