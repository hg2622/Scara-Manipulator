function [result, distance]= jacobian_inverse(q, xe_dot_dot)
   
    a1 = 0.5; 
    a2 = 0.5;

    
  
    theta1 = q(1);  
    theta2 = q(2);  
    d3 = q(3);
    
    % record the threshold
    d3_min = 0.25; 
    d3_max = 1;

    % Jacobian matrix
    J = [
        -a1 * sin(theta1) - a2 * sin(theta1 + theta2), -a2 * sin(theta1 + theta2), 0, 0;
         a1 * cos(theta1) + a2 * cos(theta1 + theta2),  a2 * cos(theta1 + theta2), 0, 0;
         0, 0, -1, 0;
         1, 1, 0, 1
    ]; 


    P_end_effector = [
        a1 * cos(theta1) + a2 * cos(theta1 + theta2);
        a1 * sin(theta1) + a2 * sin(theta1 + theta2);
        1 - d3
    ];

    obs_center = [0.4; -0.7; 0.5];

    % calculate the gradient
    distance = norm(P_end_effector - obs_center);
    dw_dq = (P_end_effector - obs_center)'/distance;
    

    % Internal motion for obstacle avoidance
    ka = 0.08 % gain
    dw_dq=ka*dw_dq;

    qa_dd =[dw_dq';0]; % Internal motion update

    

   
    qa_dot= J * qa_dd;

 % Check for d3 limits, other q value is corrected by errors, no need to check
    if d3>1 || d3<0.25
        qa_dot=[0;0;0;0];   
    end

    secondary_matrix=(eye(4) - pinv(J) * J);
    secondary_matrix(:,3)=[0;0;1;0]; % I found that I have to turn this on manually

    result = pinv(J) * xe_dot_dot + secondary_matrix * qa_dot; 
   
end
