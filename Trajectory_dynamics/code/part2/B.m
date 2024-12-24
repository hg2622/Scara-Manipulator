function [result, matrix] = B(q, q_dot_dot)
 
 
    % Extract joint variables from q
    theta1 = q(1);
    theta2 = q(2);
    d3 = q(3);
    theta4 = q(4);

    % Constants (numerical values untouched)
    a1 = 0.5; a2 = 0.5; d1 = 1;
    kr = [1, 1, 50, 20]; % Stiffness factors for motors
    lm = [25, 25, 10, 5]; % Link masses
    mm = [0.5, 0.5, 0.5, 0.5]; % Motor masses
    I = [5, 5, 0, 1]; % Link inertias
    Im = [0.0001, 0.0001, 0.01, 0.005]; % Motor inertias

    % Positional Jacobian
    Jp = [
        -a1 * sin(theta1) - a2 * sin(theta1 + theta2), -a2 * sin(theta1 + theta2), 0, 0;
         a1 * cos(theta1) + a2 * cos(theta1 + theta2),  a2 * cos(theta1 + theta2), 0, 0;
         0, 0, -1, 0;
    ];

    % Orientation Jacobian
    Jo = [
        0, 0, 0, 0;
        0, 0, 0, 0;
        1, 1, 0, 1;
    ];

    % Direct kinematics for rotational matrices
    T1_0 = [cos(theta1), -sin(theta1), 0, a1 * cos(theta1);
            sin(theta1), cos(theta1),  0, a1 * sin(theta1);
            0,           0,            1, d1;
            0,           0,            0, 1];
    T2_1 = [cos(theta2), -sin(theta2), 0, a2 * cos(theta2);
            sin(theta2), cos(theta2),  0, a2 * sin(theta2);
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

    % Rotation matrices
    R1_0 = T1_0(1:3, 1:3);
    R2_1 = T2_1(1:3, 1:3);
    R3_2 = T3_2(1:3, 1:3);
    R4_3 = T4_3(1:3, 1:3);

    R = {R1_0, R2_1 * R1_0, R3_2 * R2_1 * R1_0, R4_3 * R3_2 * R2_1 * R1_0};    

    % Initialize B matrix
    Bp = zeros(4, 4); % Positional part
    Bo = zeros(4, 4); % Orientation part

    % Compute B(q)
    for i = 1:4
        % Link kinetic
        % Positional and orientation Jacobians
        Jp_i = zeros(3, 4);
        Jp_i(:, 1:i) = Jp(:, 1:i);
        Jo_i = zeros(3, 4);
        Jo_i(:, 1:i) = Jo(:, 1:i);

        % Positional and rotational contributions
        Bp = Bp + lm(i) * (Jp_i' * Jp_i);
        Bo = Bo + [I(i) * (Jo_i') * R{i} * R{i}' * Jo_i];

        % Motor kinetic
        Jo_i(3, i) = kr(i);
        if i > 1
            Jo_i(3, 1:i-1) = Jo(3, 1:i-1);
            Jp_i(:, 1:i-1) = Jp(:, 1:i-1);
        end

        % Motor contributions
        Bp = Bp + mm(i) * (Jp_i' * Jp_i);
        Bo = Bo + [Im(i) * Jo_i' * R{i} * R{i}' * Jo_i]; 

        matrix = Bp + Bo;
    end

    result = matrix * q_dot_dot;
end
