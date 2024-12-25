function [matrix_sym] = B_partial(q_sym)
    % Define symbolic variables for q 
    syms theta1_sym theta2_sym d3_sym theta4_sym

    % Extract symbolic joint variables
    theta1_sym = q_sym(1);
    theta2_sym = q_sym(2);
    d3_sym = q_sym(3);
    theta4_sym = q_sym(4);

    % Constants stays numeric
    a1 = 0.5; a2 = 0.5; d1 = 1;
    kr = [1, 1, 50, 20]; % Stiffness factors for motors
    lm = [25, 25, 10, 5]; % Link masses
    mm = [0.5, 0.5, 0.5, 0.5]; % Motor masses
    I = [5, 5, 0, 1]; % Link inertias
    Im = [0.0001, 0.0001, 0.01, 0.005]; % Motor inertias

    % Positional Jacobian
    Jp_sym = [
        -a1 * sin(theta1_sym) - a2 * sin(theta1_sym + theta2_sym), -a2 * sin(theta1_sym + theta2_sym), 0, 0;
         a1 * cos(theta1_sym) + a2 * cos(theta1_sym + theta2_sym),  a2 * cos(theta1_sym + theta2_sym), 0, 0;
         0, 0, -1, 0;
    ];

    % Orientation Jacobian
    Jo_sym = [
        0, 0, 0, 0;
        0, 0, 0, 0;
        1, 1, 0, 1;
    ];

    % Symbolic Direct Kinematics
    T1_0_sym = [cos(theta1_sym), -sin(theta1_sym), 0, a1 * cos(theta1_sym);
                sin(theta1_sym), cos(theta1_sym),  0, a1 * sin(theta1_sym);
                0,               0,               1, d1;
                0,               0,               0, 1];
    T2_1_sym = [cos(theta2_sym), -sin(theta2_sym), 0, a2 * cos(theta2_sym);
                sin(theta2_sym), cos(theta2_sym),  0, a2 * sin(theta2_sym);
                0,               0,               1, 0;
                0,               0,               0, 1];
    T3_2_sym = [1, 0, 0, 0;
                0, 1, 0, 0;
                0, 0, 1, d3_sym;
                0, 0, 0, 1];
    T4_3_sym = [cos(theta4_sym), -sin(theta4_sym), 0, 0;
                sin(theta4_sym), cos(theta4_sym),  0, 0;
                0,               0,               1, 0;
                0,               0,               0, 1];

    % Symbolic Rotation Matrices
    R1_0_sym = T1_0_sym(1:3, 1:3);
    R2_1_sym = T2_1_sym(1:3, 1:3);
    R3_2_sym = T3_2_sym(1:3, 1:3);
    R4_3_sym = T4_3_sym(1:3, 1:3);

    R_sym = {
        R1_0_sym,
        R2_1_sym * R1_0_sym,
        R3_2_sym * R2_1_sym * R1_0_sym,
        R4_3_sym * R3_2_sym * R2_1_sym * R1_0_sym
    };

    
    Bp_sym = zeros(4, 4, 'sym'); 
    Bo_sym = zeros(4, 4, 'sym'); 

    
    %this part is the exact same as B function, just symbolic one
   for i = 1:4
    
    Jp_i_sym = sym(zeros(3, 4));
    Jo_i_sym = sym(zeros(3, 4));

   
    Jp_i_sym(:, 1:i) = Jp_sym(:, 1:i);
    Jo_i_sym(:, 1:i) = Jo_sym(:, 1:i);

    Bp_sym = Bp_sym + lm(i) * (Jp_i_sym' * Jp_i_sym);
    Bo_sym = Bo_sym + I(i) * (Jo_i_sym' * R_sym{i} * R_sym{i}' * Jo_i_sym);

   
    Jo_i_sym(3, i) = kr(i);
    if i > 1
       
        Jo_i_sym(3, 1:i-1) = Jo_sym(3, 1:i-1);
        Jp_i_sym(:, 1:i-1) = Jp_sym(:, 1:i-1);
    end

    Bp_sym = Bp_sym + lm(i) * (Jp_i_sym' * Jp_i_sym);
    Bo_sym = Bo_sym + I(i) * (Jo_i_sym' * R_sym{i} * R_sym{i}' * Jo_i_sym);



   end

   
    B_matrix = Bp_sym + Bo_sym;
  

    % Initialize Hijk as a 4x4x4 symbolic tensor


    H = sym(zeros(4, 4, 4));

    % Loop over all indices i, j, k
    for i = 1:4
        for j = 1:4
            for k = 1:4
                % Compute partial derivatives
                dBij_dqk = diff(B_matrix(i, j), q_sym(k)); 
                dBjk_dqi = diff(B_matrix(j, k), q_sym(i)); 
                
                % Calculate Hijk
                H(i, j, k) = dBij_dqk - 0.5 * dBjk_dqi;
            end
        end
    end

    for b=1:4
        
Hjk = H(b, :, :); 
Hjk = reshape(Hjk, [4, 4]);
disp(b)
disp(simplify(Hjk));  % I will use this line's output to the C(q,q_dot) in n.m implementation

    end


end


syms q1 q2 d3 q4;
assumeAlso([q1, q2], 'real');
B_partial([q1 q2 d3 q4]);