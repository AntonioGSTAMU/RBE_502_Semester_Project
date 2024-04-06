clc; clear; close all

addpath('./src');

% QUADROTOR

g = 9.81;  % The gravitational acceleration [m/s^2]
l = 0.2;  % Distance from the center of mass to each rotor [m]
m = 0.5;  % Total mass of the quadrotor [kg]
I = [1.24, 1.24, 2.48];  % Mass moment of inertia [kg m^2]
mu = 3.0;  % Maximum thrust of each rotor [N]
sigma = 0.01;  % The proportionality constant relating thrust to torque [m]
p=[g,l,m,sigma];
quad = quadrotor(g, l, m, diag(I), mu, sigma);
n=[0,0,0]';
r=[0,0,0]';

%[A, B] = linearize(p, I, n, r);
A = [0	0	0	0	0	0	1	0	0	0	0	0;
     0	0	0	0	0	0	0	1	0	0	0	0;
     0	0	0	0	0	0	0	0	1	0	0	0;
     0	0	0	0	0	0	0	0	0	1	0	0;
     0	0	0	0	0	0	0	0	0	0	1	0;
     0	0	0	0	0	0	0	0	0	0	0	1;
     0	0	0	0	19.62	0	0	0	0	0	0	0;
     0	0	0	-19.62	0	0	0	0	0	0	0	0;
     0	0	0	0	0	0	0	0	0	0	0	0;
     0	0	0	0	0	0	0	0	0	0	0	0;
     0	0	0	0	0	0	0	0	0	0	0	0;
     0	0	0	0	0	0	0	0	0	0	0	0];
B = [0                           0                           0                           0;
     0                           0                           0                           0;
     0                           0                           0                           0;
     0                           0                           0                           0;
     0                           0                           0                           0;
     0                           0                           0                           0;
     0                           0                           0                           0;
     0                           0                           0                           0;
     2                           2                           2                           2;
     0                           0.0203873598369011         0                          -0.0203873598369011;
     -1                          0                           1                           0;
     0.0200000000000000        -0.0200000000000000         0.0200000000000000        -0.0200000000000000];


% INTRUDER
path = @(t) [2*cos(0.5*t); 0.5*sin(0.5*t); 2];
dist = struct("r", @(t,z)0.1*[sin(t); sin(2*t); sin(4*t)],...
    "n", @(t,z) 0.1*[0.1; 0.01; 0.1]);
 
intruder = uav(path, dist);
Q = diag([1000,1000, 1000, 100, 100, 100, 100, 100, 100, 10000, 10000, 10000]);
R = eye(4)*0.1*100 ;
% CONTROLLER
%target=[1,1,2]';

ctrl = lqr_control(quad, A,B,Q,R);

% SIMULATION

sim = simulator(quad, ctrl, intruder);
sim.simtime = [0 20];
sim.timestep = 0.01;
sim.epsilon = 0.1;

z0 = zeros(12,1);

[t,z,u,d,y] = sim.simulate(z0);

% ANIMATION
sim.animate(t, z, y);

