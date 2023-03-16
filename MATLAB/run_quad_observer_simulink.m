clear
clc
% Tim McNamara, Atharv Sathe
% 18776


% Run the Simulink model for the linear observer
% Option to add noise to the measurement values

rng(2);
run('quadrocopter_LQR.m')
ctrlr_poles = eig(A-B*K);
obsv_poles = 3*ctrlr_poles;

% C = eye(12); % if using full set of measurements
C = zeros(9,12); % if using reduced measurements
C(1,2) = 1;
C(2,4) = 1;
C(3,6) = 1;
C(4:9,7:12) = eye(6);

L = place(A', C', obsv_poles)';

L_file = "L_reduced.txt";
C_file = "C_reduced.txt";
writematrix(L,L_file, 'Delimiter','tab');
writematrix(C,C_file, 'Delimiter','tab');

x0 = zeros(12,1);
% run2 values (run 1 was 15, 15 15)
px_init = -15; py_init = -15; pz_init = -15;
x0(2) = px_init;
x0(4) = py_init;
x0(6) = pz_init;

xhat0 = x0;
xext0 = [x0;xhat0];

sensor_noise_variance = [.1;.2;.1;.2;.1;.2;.01;.02;.01;.02;.01;.02];
sensor_noise_variance = sensor_noise_variance * 0; % zero out noise to test velocity estimator

T_final = 20;

nl_update = true; % set true if you want observer to use nonlinear dynamics update

sim_out = sim('quadrotor_linear_obsv_model_submission',(0:0.01:T_final));
x = sim_out.yout{1}.Values.Data;
t = sim_out.yout{1}.Values.Time;
xhat = sim_out.yout{2}.Values.Data;
plot_pos = false;
plot_velocity = true;
plot_angle = false;
plot_angvel = false;
plot_input = false;
legend_labels = ["true state"; "estimated state"];
plot_quadrotor_model(x,t, [], xhat, t, [], plot_pos, plot_velocity, plot_angle, plot_angvel, plot_input, legend_labels)
sgtitle("SIMULINK: Estimated positional velocities using reduced measurement set")