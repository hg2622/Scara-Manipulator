function J_dot_q_dot = jacobian_dot_(q, q_dot)
    a1 = 0.5; 
    a2 = 0.5;


    theta1 = q(1);
    theta2 = q(2);
   
    theta1_dot = q_dot(1);
    theta2_dot = q_dot(2);

    J_dot = [
        -a1 * cos(theta1) * theta1_dot - a2 * cos(theta1 + theta2) * (theta1_dot + theta2_dot), -a2 * cos(theta1 + theta2) * (theta1_dot + theta2_dot), 0,  0;
        -a1 * sin(theta1) * theta1_dot - a2 * sin(theta1 + theta2) * (theta1_dot + theta2_dot), -a2 * sin(theta1 + theta2) * (theta1_dot + theta2_dot), 0,  0;
        0, 0, 0, 0;
        0, 0, 0, 0;
    ];

   
    J_dot_q_dot = J_dot * q_dot;
end
