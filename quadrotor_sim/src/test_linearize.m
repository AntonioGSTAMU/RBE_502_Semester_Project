clc; clear; close all

g = 9.81;   % gravitational acceleration [m/s^2]
l = 0.2;    % distance from the center of mass to each rotor [m]
m = 0.5;    % total mass of the quadrotor [kg]
I = [1.24, 1.24, 2.48]; % mass moment of inertia [kg m^2]
mu = 3.0;   % maximum thrust of each rotor [N]
sigma = 0.01; % proportionality constant relating thrust to torque [m]
I11 = 1.24;
I22 = 1.24;
I33 = 2.48;
p = [g, l, m, sigma];

n = [0, 0, 0]';
r = [0, 0, 0]';

% [A, B] = linearize_quadrotor_symbolic(p, I, n, r);
[A, B] = linearize(p, I, n, r);


