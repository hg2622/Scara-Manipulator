function [p_d, theta_d, pd_theta_d] = direct_kin(q)
 % extract d3 and thetas
    theta1 = q(1);
    theta2 = q(2);
    d3 = -q(3);
    theta4 = q(4);
    
    % as required in the project manual
    a1 = 0.5;
    a2 = 0.5;
    d1 = 1;

    % each transformation matrices, for example, T1_0 map frame 1 to 0
    T1_0 = [cos(theta1), -sin(theta1), 0, a1*cos(theta1);
            sin(theta1), cos(theta1),  0, a1*sin(theta1);
            0,           0,            1, d1;
            0,           0,            0, 1];
    
    T2_1 = [cos(theta2), -sin(theta2), 0, a2*cos(theta2);
            sin(theta2), cos(theta2),  0, a2*sin(theta2);
            0,           0,            1, 0;
            0,           0,            0, 1];
    
    T3_2 = [1, 0, 0, 0;
            0, 1, 0, 0;
            0, 0, 1, d3;
            0, 0, 0, 1];
    
    T4_3 = [cos(theta4), -sin(theta4), 0, 0;
            sin(theta4), cos(theta4),  0, 0;
            0,           0,            1, 0;
            0,           0,            0, 1];
    
    T_end_effector = T1_0 * T2_1 * T3_2 * T4_3;
    
    p_d = T_end_effector(1:3, 4)';
    
    theta_d = theta1 + theta2 + theta4;
    
    pd_theta_d = [p_d'; theta_d];
end
