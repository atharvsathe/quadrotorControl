% Tim McNamara
% 18-776 Project Part 1

clear all
clc

% parameters
global m l k1 k2 rotor_time_constant Jx Jy Jz RgToc M g
m = 0.6; % kg
l = 0.2159/2; % m
k1 = 4.0; % rotor_thrust
k2 = 0.05; % rotor_torque
rotor_time_constant = 0.005;
Jx = 0.0092;
Jy = 0.0091;
Jz = 0.0101;
g = 9.8;

% rotation of gimbal coordinates to camera coordinates
RgToc = [0 1 0;
         0 0 1;
         1 0 0];
     
% big M matrix mapping motor command signals to forces and torques
% [F;torque] = M*delta
% delta = inv(M)*[F;torque] - but to saturate at 0,1 --> 
% delta = min(ones(4), max(delta, zeros(4))
M = [k1    k1    k1    k1;
     0    -l*k1  0     l*k1;
     l*k1  0    -l*k1  0;
     -k2   k2   -k2    k2];

A = zeros(12);
B = zeros(12,4);

% Linearize f(t,x,u) around equilibrium point x = 0, u = [mg, 0, 0, 0]
A(2,1) = 1; % d/dt (x) = xdot
A(4,3) = 1; % d/dt (y) = ydot
A(6,5) = 1; % d/dt (z) = zdot

A(8,7) = 1; % d/dt (phi) = p
A(10,9) = 1; % d/dt (theta) = q
A(12,11) = 1; % d/dt (psi) = r

A(1,10) = -g; %  xddot = theta*az = theta*-m*g/m @ eq
A(3,8) = g; %  yddot = -phi*az =  -phi*-m*g/m @ eq

B(5, 1) = -1/m; % zddot = az = -F/m
B(7, 2) =  1/Jx;
B(9, 3) =  1/Jy;
B(11,4) =  1/Jz;

disp("Rank of linearized controllability matrix");
disp(rank(ctrb(A,B)));
disp("System is controllable --> it is stabilizable");

% Want to keep angles small so small angle approximation holds (phi, theta
% <15º) --> Use Bryson's method to choose Q values
Q = .0025*eye(12);
% Max allowable positions
% Q(2,2) = 1/10^2;
% Q(4,4) = 1/10^2;
% Q(6,6) = 1/10^2;
% Max allowable angles
phi_max = 10*pi/180;
theta_max = 10*pi/180;
psi_max = 180*pi/180;
Q(8,8) = 1/phi_max^2;
Q(10,10) = 1/theta_max^2;
Q(12,12) = 1/psi_max^2;

% Max allowable angular rates
omega_max = 1800*pi/180; % rad/s
Q(7,7) = 1/omega_max^2;
Q(9,9) = 1/omega_max^2;
Q(11,11) = 1/omega_max^2;

% The largest input we could realistically put in is a PWM signal of 1.0 to
% each motor. We can use M to find the maximum realistic force and torques
% that could be applied
safety_factor = 1;
F_max = safety_factor*16; % be conservative
tau_xy_max = safety_factor*.4318;%norm(M(2:end,:), inf) * .5; % be conservative
tau_z_max = safety_factor*.1;
R = eye(4);
R(1,1) = 1/F_max^2;
R(2,2) = 1/tau_xy_max^2;
R(3,3) = 1/tau_xy_max^2;
R(4,4) = 1/tau_z_max^2;

K = lqr(A, B, Q, R);
%K_file = "~/px4-19/Firmware/src/modules/mc_att_control/lqr_files/tgmK.txt";
K_file = "/Users/tmcnama2/px4-19/Firmware/build/px4_sitl_default/tmp/rootfs/tgmK.txt";

writematrix(K,K_file,'Delimiter','tab')

P = lyap(transpose(A-B*K), eye(12));
%P_file = "~/px4-19/Firmware/src/modules/mc_att_control/lqr_files/tgmP.txt";
P_file = "/Users/tmcnama2/px4-19/Firmware/build/px4_sitl_default/tmp/rootfs/tgmP.txt";

writematrix(P,P_file,'Delimiter','tab')

% x0 = zeros(12,1);
% % run2 values (run 1 was 15, 15 15)
% x0(2) = -15;
% x0(4) = -15;
% x0(6) = 15;

% quadrotorsys = @(t,x) quadrotor_exact(t,x,K);
% [t, x] = ode45(quadrotorsys, [0, 30], x0);
% u_norm = zeros(4,length(t));
% u = zeros(4,length(t));
% for i = 1:length(t)
%     [u(:,i), u_norm(:,i)] = lqr_control(x(i,:), K);
% end

% x_true = readmatrix('run2_state.txt');
% t_true = (x_true(:,1) - x_true(1,1))/1000000; % convert to relative seconds
% x_true = x_true(:,2:end);
% % correct for set points
% x_true(:,2) = x_true(:,2) - 15;
% x_true(:,4) = x_true(:,4) - 15;
% x_true(:,6) = x_true(:,6) + 15;
% 
% plot_quadrotor_model(x,t, u, x_true, t_true, u_norm, true, true, true, true, true)

% % plot 
% u_norm = zeros(4,length(t));
% for i = 1:length(t)
%     u_norm(:,i) = lqr_control(x(i,:), K);
% end
% 
% figure()
% for i = 1:4
%     subplot(4,1,i)
%     plot(t, u_norm(i,:));
%     ylim([-1.5, 1.5]);
% end

% Vehicle frame: origin is center of mass of vehicle, but axes are aligned
% with the axis of the intertial frame Fi (i North, j East, k into Earth)
% Vehicle-1 frame - vehicle frame positively rotated about k_v by yaw angle
% psi --> x_v1 now always points out of the nose of the aircraft
function R = RvTov1(psi)
    R = [cos(psi) sin(psi) 0;
         -sin(phi) cos(psi) 0;
             0        0  1];
end

% Vehicle2 frame: vehicle-1 frame right-hand-rotated about j_v1 by pitch
% angle theta --> j_v2 now always points out of the right wing of aircraft
function R = Rv1Tov2(theta)
    R = [cos(theta) 0 -sin(theta);
           0    1     0;
         sin(theta) 0  cos(theta)];
end

% Body frame - vehicle-2 frame right-hand-rotated about i_v2 by roll angle
% phi, so no k_b points out of the belly of the vehicle
function R = Rv2Tob(phi)  
    R = [1          0      0;
         0      cos(phi) sin(phi);
         0     -sin(phi) cos(phi)];
end

% vehicle-2 coordinates to body coordinates
function R = RvTob(psi, theta, phi)
    R = Rv2Tob(phi)*Rv1Tov2(theta)*RvTov1(psi);
end

% vehicle-1 coordinates to body coordinates
function R = Rv1Tob(theta, phi)
    R = Rv1Tov2(theta)*Rv2Tob(phi);
end

% relative position coordinates to camera frame
function [u, u_norm] = lqr_control(x,K)
    u = -K*x';
    u(1,:) = u(1,:) + 5.886;
    ff_thrust = 5.886;
    u_norm = zeros(4,1);
    u_norm(1) = min([max([(u(1)+ff_thrust)/16, 0.0]), 1.0]);
    u_norm(2) = min([max([(u(2))/(0.1080*4.0), -1.0]), 1.0]);  
    u_norm(3) = min([max([(u(3))/(0.1080*4.0),  -1.0]), 1.0]);
    u_norm(4) = min([max([(u(4))/(0.1*1.0), -1.0]), 1.0]);
end

