function output = n(q, q_dot)
    % Extract d3 and thetas
    theta1 = q(1);
    theta2 = q(2);
    d3 = -q(3);
    theta4 = q(4);
    
    % As required in the project manual
    a1 = 0.5;
    a2 = 0.5;
    d1 = 1;

    mm = [0, 0, 0, 0]; % Motor masses not provided, assume no mass
    lm = [25, 25, 10, 5]; % Link masses
    F = [0.0001, 0.0001, 0.01, 0.005]; % Friction coefficients

    % Transformation Matrices
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
    
    % Compute the full transformations
    T1 = T1_0;
    T2 = T2_1 * T1_0;
    T3 = T3_2 * T2_1 * T1_0;
    T4 = T4_3 * T3_2 * T2_1 * T1_0;

    % Extract position vectors of the centers of mass
    P1 = T1(:, 4);
    P2 = T2(:, 4); 
    P3 = T3(:, 4); 
    P4 = T4(:, 4);

    P = {P1, P2, P3, P4}; % Store in a cell array

    % Compute gravity contributions
    G = zeros(1, 4); % Initialize gravitational torque vector
    for i = 1:4
        G(i) = 9.8 * (mm(i) + lm(i)) * P{i}(3); % Use the z-coordinate for gravity
        P{i}(3);
    end


 % C(q, q_dot) contribution

    %here is all the Hijk I use another function to calculate:

  H = zeros(4, 4, 4);
H(1,:, :) = [
    0,    -65*sin(theta2),   0, 0;
    0,   -20*sin(theta2),   0, 0;
    0,      0,    0, 0;
    0,      0,   0, 0
];
H(2,:,:) = [
    65*sin(theta2)/2, -10*sin(theta2), 0, 0;
    10*sin(theta2),           0, 0, 0;
              0,           0, 0, 0;
              0,           0, 0, 0
];
H(:, :, 3) = zeros(4, 4);
H(:, :, 4) = zeros(4, 4);



C = zeros(4, 4);

% Compute C_ij
for i = 1:4
    % Sum over k for each (i, j)
    for j = 1:4
        for k = 1:4
            C(i, j) = C(i, j) + H(i, j, k) * q_dot(k);
        end
    end
end

   
%calculate the final result by summing terms

    output = G + F .* q_dot' + (C*q_dot)';
    output= output';
end




% check if make sense
%q = [4, 4, 0.5, 1];
%q_dot = [1, 1, 1, 1];
%a = n_(q, q_dot);
%disp(a);
