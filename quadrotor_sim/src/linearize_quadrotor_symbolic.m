function [A, B] = linearize_quadrotor_symbolic(p, I_nu, n_nu, r_nu)
    % Define symbolic variables and parameters
    syms x1 x2 x3 phi theta pssi v1 v2 v3 omega1 omega2 omega3 real
    z = [x1, x2, x3, phi, theta, pssi, v1, v2, v3, omega1, omega2, omega3];

    % Define other symbolic variables for system inputs and disturbances
    syms I11 I22 I33 n1 n2 n3 r1 r2 r3 u1 u2 u3 u4 real
    I = diag([I11, I22, I33]);
    n = [n1, n2, n3];
    r = [r1, r2, r3];
    u = [u1, u2, u3, u4];

    % Physical and inertial parameters are represented symbolically
    p = sym('p', [1, 4], 'real');
    I_nu = sym('I_nu', [1, 3], 'real');
    n_nu = sym('n_nu', [1, 3], 'real');
    r_nu = sym('r_nu', [1, 3], 'real');

    % Rotation matrix from body-fixed frame C to inertial frame E
    Rot_CE = [cos(theta)*cos(pssi), sin(phi)*sin(theta)*cos(pssi) - cos(phi)*sin(pssi), sin(phi)*sin(pssi) + cos(phi)*sin(theta)*cos(pssi);
              cos(theta)*sin(pssi), cos(phi)*cos(pssi) + sin(phi)*sin(theta)*sin(pssi), cos(phi)*sin(theta)*sin(pssi) - sin(phi)*cos(pssi);
              -sin(theta),         sin(phi)*cos(theta),                            cos(phi)*cos(theta)];

    % Transformation matrix T_inv
    T_inv = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
             0, cos(phi),            -sin(phi);
             0, sin(phi)/cos(theta), cos(phi)/cos(theta)];

    % Define the dynamics of the quadrotor system symbolically
    f = [[v1; v2; v3];
         T_inv * [omega1; omega2; omega3];
         -Rot_CE'*[0, 0, p(1)]' + (1/p(3)) * Rot_CE * [0; 0; u(1)+u(2)+u(3)+u(4)] +  (1/p(3)) * Rot_CE * [r(1); r(2); r(3)];
         inv(I) * ([(u(2) - u(4)) * p(2); (u(3) - u(1)) * p(2); (u(1) - u(2) + u(3) - u(4)) * p(4)] + [n(1); n(2); n(3)] - cross([omega1; omega2; omega3], I * [omega1; omega2; omega3]))];
         
    % Linearize the system by calculating the Jacobian matrices symbolically
    A = jacobian(f, z);
    A = simplify(A);
    B = jacobian(f, u);
    B = simplify(B);

    % Display Jacobian matrices A and B in symbolic form
    disp("Jacobian matrix A (Symbolic):");
    disp(A);
    disp("Jacobian matrix B (Symbolic):");
    disp(B);
end
