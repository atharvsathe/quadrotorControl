clear
clc
% Tim McNamara, Atharv Sathe
% 18776

% Run the Simulink model for full-state controller
% Option to add noise to the measurement values

rng(2);
run('quadrocopter_LQR.m')

x0 = zeros(12,1);
% run2 values (run 1 was 15, 15 15)
px_init = -15; py_init = -15; pz_init = -15;
x0(2) = px_init;
x0(4) = py_init;
x0(6) = pz_init;

xhat0 = x0;
xext0 = [x0;xhat0];

sensor_noise_variance = [.1;.2;.1;.2;.1;.2;.01;.02;.01;.02;.01;.02];
%sensor_noise_variance = sensor_noise_variance*0; % zero out noise to test velocity estimator

T_final = 20;

sim_out = sim('quadrotor_model_submission',(0:0.01:T_final));
x = sim_out.yout{1}.Values.Data;
t = sim_out.yout{1}.Values.Time;

y = zeros(size(x));
for i = 1:12
    y(:,i) = squeeze(sim_out.yout{2}.Values.Data(i,1,:));
end
plot_pos = true;
plot_velocity = true;
plot_angle = true;
plot_angvel = true;
plot_input = false;
legend_labels = ["true state"; "noisy measured state"];
plot_quadrotor_model(x,t, [], y, t, [], plot_pos, plot_velocity, plot_angle, plot_angvel, plot_input, legend_labels)
