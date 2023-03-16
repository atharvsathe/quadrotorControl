clear
clc
close 
% Tim McNamara, Atharv Sathe
% 18776

% Tune the complementary filter parameters to have an attitude estimation
% that matches the EKF attitude values as closely as possible
% Sensor data taken from a run where vehicle was commanded to 
% (10, -5, -10, 10 deg) for 10 seconds and then to (-15, 10, -15, -10 deg)

%sensor_data_file = "~/px4-19/Firmware/build/px4_sitl_default/tmp/rootfs/sensor_data.txt";
sensor_data_file = "sensor_data.txt";
sensor_data = readmatrix(sensor_data_file);
N = size(sensor_data,1);

t_sensor = (sensor_data(:,1) - sensor_data(1,1))/1e6;
mag_x = sensor_data(:,2);
mag_y = sensor_data(:,3);
mag_z = sensor_data(:,4);

imu_p = sensor_data(:,5);
imu_q = sensor_data(:,6);
imu_r = sensor_data(:,7);
imu_dt = sensor_data(:,8);

% seeing something very weird here
% it looks as though acc_y is really acc_z
acc_x = sensor_data(:,9);
acc_y = sensor_data(:,10);
acc_z = sensor_data(:,11);
acc_dt = sensor_data(:,12);

p_hat = zeros(N,1);
q_hat = zeros(N,1);
r_hat = zeros(N,1);
th_hat = zeros(N,1);
phi_hat = zeros(N,1);
psi_hat = zeros(N,1);
mx = zeros(N,1);
my = zeros(N,1);
mz = zeros(N,1);
ax = zeros(N,1);
ay = zeros(N,1);
az = zeros(N,1);

% lpf weights - closer to 1 means more weight given to newest measurement
acc_lpf_weight = 1;
imu_lpf_weight = .98;
mag_lpf_weight = 1;
% complementary filter weights - closer to 1 means more weight given to
% gyro data versus accelerometer/magnetometer
accel_weight = 0.001;
gyro_weight = 1-accel_weight;
mag_weight = .01;
gyro_weight_heading = 1-mag_weight;
cf_use_filtered_data = false;

for i = 2:N
    % LPF the data where applicable
    mx(i) = (1-mag_lpf_weight)*mx(i-1) + mag_lpf_weight*mag_x(i);
    my(i) = (1-mag_lpf_weight)*my(i-1) + mag_lpf_weight*mag_y(i);
    mz(i) = (1-mag_lpf_weight)*mz(i-1) + mag_lpf_weight*mag_z(i);
    
    p_hat(i) = (1-imu_lpf_weight)*p_hat(i-1) + imu_lpf_weight*imu_p(i);
    q_hat(i) = (1-imu_lpf_weight)*q_hat(i-1) + imu_lpf_weight*imu_q(i);
    r_hat(i) = (1-imu_lpf_weight)*r_hat(i-1) + imu_lpf_weight*imu_r(i);
    
    ax(i) = (1-acc_lpf_weight)*ax(i-1) + acc_lpf_weight*acc_x(i);
    ay(i) = (1-acc_lpf_weight)*ay(i-1) + acc_lpf_weight*acc_y(i);
    az(i) = (1-acc_lpf_weight)*az(i-1) + acc_lpf_weight*acc_z(i);
    
    % estimate using the complementary filters
    %dt = t_sensor(i) - t_sensor(i-1);
    dt = imu_dt(i);
    roll_from_acc =  atan2(ay(i), sqrt(ax(i)^2 + az(i)^2));
    pitch_from_acc = atan2(-ax(i), sqrt(ay(i)^2 + az(i)^2));
    
    if cf_use_filtered_data
        cf_p = p_hat(i); 
    else
        cf_p = imu_p(i);
    end
    phi_hat(i) = gyro_weight*(phi_hat(i-1) + cf_p*dt) + accel_weight*roll_from_acc; 
    
    if cf_use_filtered_data
        cf_q = q_hat(i); 
    else
        cf_q = imu_q(i);
    end
    th_hat(i) = gyro_weight*(th_hat(i-1) + cf_q*dt) + accel_weight*pitch_from_acc;
    
    mag_len = sqrt(mx(i)^2 + my(i)^2 + mz(i)^2);
    mx_norm = mx(i)/mag_len;
    my_norm = my(i)/mag_len;
    mz_norm = mz(i)/mag_len;
    
    num = sin(phi_hat(i))*mz_norm - cos(phi_hat(i))*my_norm;
    denom = cos(th_hat(i))*mx_norm + sin(phi_hat(i))*sin(th_hat(i))*my_norm + cos(th_hat(i))*sin(phi_hat(i))*mz_norm;
    psi_from_mag = atan2(num, denom);
    %psi_from_mag = -atan2(my_norm, mx_norm);
    cf_r = psi_hat(i); % could also use imu_r(i) if you don't want filtered value
    psi_hat(i) = gyro_weight_heading*(psi_hat(i-1) + cf_r*imu_dt(i)) + mag_weight*psi_from_mag;
end

% ground truth
% state_data_file = "~/px4-19/Firmware/build/px4_sitl_default/tmp/rootfs/state.txt";
state_data_file = "sensor_data_true_state.txt";
state_data = readmatrix(state_data_file);
t_true = (state_data(:,1)-state_data(1,1))/1e6;
phi_true = state_data(:,9);
th_true = state_data(:,11);
psi_true = state_data(:,13);
p_true = state_data(:,8);
q_true = state_data(:,10);
r_true = state_data(:,12);

% Attitude angle plots
figure()
subplot(3,1,1)
plot(t_true, phi_true.*180/pi, t_sensor, phi_hat.*180/pi);
xlim([0 22])
grid on
ylabel('degrees º')
xlabel('time (s)')
title('\phi')
legend('\phi ekf', '\phi comp filt')

subplot(3,1,2)
plot(t_true, th_true.*180/pi, t_sensor, th_hat.*180/pi);
xlim([0 22])
grid on
ylabel('degrees º')
xlabel('time (s)')
title('\theta')
legend('\theta ekf', '\theta comp filt')

subplot(3,1,3)
plot(t_true, psi_true.*180/pi, t_sensor, psi_hat.*180/pi);
xlim([0 22])
grid on
ylabel('degrees º')
xlabel('time (s)')
title('\psi')
legend('\psi ekf', '\psi comp filt')

figure()
subplot(3,1,1)
plot(t_true, (phi_true-phi_hat)*180/pi);
ylim([-5 5])
xlim([0 22])
grid on
ylabel('degrees º')
title('\phi_{ekf} - \phi_{cf}')
subplot(3,1,2)
plot(t_true, (th_true-th_hat)*180/pi);
ylim([-5 5])
xlim([0 22])
ylabel('degrees º')
grid on
title('\theta_{ekf} - \theta_{cf}')
subplot(3,1,3)
plot(t_true, (psi_true-psi_hat)*180/pi);
xlim([0 22])
ylim([-5 5])
ylabel('degrees º')
grid on
title('\psi_{ekf} - \psi_{cf}')

% Angular rate errors
figure()
subplot(3,1,1)
plot(t_true, p_true.*180/pi, t_sensor, p_hat.*180/pi);
xlim([0 22])
ylim([-20 20])
grid on
ylabel('degrees º./s')
xlabel('time (s)')
title('p')
legend('p ekf', 'p comp filter')

subplot(3,1,2)
plot(t_true, q_true.*180/pi, t_sensor, q_hat.*180/pi);
xlim([0 22])
ylim([-20 20])
grid on
ylabel('degrees º./s')
xlabel('time (s)')
title('q')
legend('q ekf', 'q comp filter')

subplot(3,1,3)
plot(t_true, r_true.*180/pi, t_sensor, r_hat.*180/pi);
xlim([0 22])
ylim([-20 20])
grid on
ylabel('degrees º./s')
xlabel('time (s)')
title('rp')
legend('r ekf', 'r comp filter')

figure()
subplot(3,1,1)
plot(t_true, (p_true-p_hat)*180/pi);
xlim([0 22])
ylim([-15 15])
grid on
ylabel('degrees º')
xlabel('time (s)')
title('p_{ekf} - p_{cf}')
subplot(3,1,2)
plot(t_true, (q_true-q_hat)*180/pi);
xlim([0 22])
ylim([-15 15])
grid on
ylabel('degrees º/s')
xlabel('time (s)')
title('q_{ekf} - q_{cf}')
subplot(3,1,3)
plot(t_true, (r_true-r_hat)*180/pi);
xlim([0 22])
ylim([-15 15])
grid on
ylabel('degrees º/s')
xlabel('time (s)')
title('r_{ekf} - r_{cf}')


