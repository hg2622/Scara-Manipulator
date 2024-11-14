function J_dot_q_dot = jacobian_dot_(q, q_dot)
    % Define link lengths
    a1 = 0.5; 
    a2 = 0.5;

    % Extract joint variables
    theta1 = q(1);
    theta2 = q(2);
    
    % Extract joint velocities
    theta1_dot = q_dot(1);
    theta2_dot = q_dot(2);

    % Time derivative of the Jacobian matrix (J_dot)
    J_dot = [
        -a1 * cos(theta1) * theta1_dot - a2 * cos(theta1 + theta2) * (theta1_dot + theta2_dot), -a2 * cos(theta1 + theta2) * (theta1_dot + theta2_dot), 0, 0;
        -a1 * sin(theta1) * theta1_dot - a2 * sin(theta1 + theta2) * (theta1_dot + theta2_dot), -a2 * sin(theta1 + theta2) * (theta1_dot + theta2_dot), 0, 0;
        0, 0, 0, 0;
        0, 0, 0, 0
    ];

    % Multiply J_dot by q_dot
    J_dot_q_dot = J_dot * q_dot; % Ensure q_dot is a column vector
end
