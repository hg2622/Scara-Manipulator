function [p_d] = direct_kin(q)
    % Inputs:
    % q: 4-element array [theta1, theta2, d3, theta4]
    theta1 = q(1); % Joint 1 angle
    theta2 = q(2); % Joint 2 angle
    d3 = -q(3);     % Prismatic joint displacement
  % End-effector rotation angle
    
    % Define robot link lengths (a1, a2) and other relevant DH parameters
    a1 = 0.5;  % Example value for link 1 length (modify as necessary)
    a2 = 0.5;  % Example value for link 2 length (modify as necessary)
    d1 = 1;  % Offset along z for the first joint (modify as necessary)

    % Transformation matrix from base to joint 1
    T1_0 = [cos(theta1), -sin(theta1), 0, a1*cos(theta1);
            sin(theta1), cos(theta1),  0, a1*sin(theta1);
            0,           0,            1, d1;
            0,           0,            0, 1];
    
    % Transformation matrix from joint 1 to joint 2
    T2_1 = [cos(theta2), -sin(theta2), 0, a2*cos(theta2);
            sin(theta2), cos(theta2),  0, a2*sin(theta2);
            0,           0,            1, 0;
            0,           0,            0, 1];
    
    % Transformation matrix for prismatic joint (joint 3)
    T3_2 = [1, 0, 0, 0;
            0, 1, 0, 0;
            0, 0, 1, d3;
            0, 0, 0, 1];
    
   
    
    % Full transformation from base to end-effector
    T_end_effector = T1_0 * T2_1 * T3_2 ;
    
    % Extract position (x, y, z) from the transformation matrix
    p_d = T_end_effector(1:3, 4)'; % Position (x, y, z)
    
  

end
