%% Attitude Estimation
clc
close all
% Load Data
load('slow_v4.mat')

input = XS2(:,2:10);
out_q = XS2(:,11:14);
% The nominal average value at Earth's surface (standard gravity)
% Thompson, Ambler, and Barry N. Taylor. "Use of the International System of Units (SI)." (2008).
g = 9.80665;

% Calculate euler angles for each iteration using ref quaernion
for i = 1:length(out_q)
    q0 = out_q(i,1);
    q1 = out_q(i,2);
    q2 = out_q(i,3);
    q3 = out_q(i,4);

    [phi, theta, psi] = quat_2_eul(q0,q1,q2,q3);
    phi_ref(i) = phi;
    theta_ref(i)=theta;
    psi_ref(i)=psi;
end

    dt      =   1/100; % Sassari Dataset Sampling Rate = 100 Hz
    t       =   0:dt:(length(input) - 1)*dt;

    acc_x   =   input(:,1);
    acc_y   =   input(:,2);
    acc_z   =   input(:,3);
    gyro_x  =   input(:,4);
    gyro_y  =   input(:,5);
    gyro_z  =   input(:,6);

    % Calculate Phi and Theta using Accelerometer Measurements
    for i = 1:length(input)
        % First Equ.
        phi1_acc(i,1)	=	atan2(acc_y(i),sqrt(acc_x(i)^2 + acc_z(i)^2));
        theta1_acc(i,1)	=	atan2(acc_x(i),sqrt(acc_y(i)^2 + acc_z(i)^2));
        % Second Equ.
        phi2_acc(i,1)   =   atan2(acc_y(i),acc_z(i));
        theta2_acc(i,1) =   atan2(acc_x(i),(acc_y(i)*sin(phi2_acc(i))+acc_z(i)*cos(phi2_acc(i))));
        theta3_acc(i,1) =   atan2(acc_x(i),g);
    end

    figure(1)
    plot(t,rad2deg(phi1_acc(:,1)),t,rad2deg(phi2_acc(:,1)),t,rad2deg(phi_ref),LineWidth = 1.2)
    legend('phi1','phi2', 'true')
    RMSE_phi_est_1 = sqrt(mean( (rad2deg(phi1_acc(:,1))-rad2deg(phi_ref(1,:))').^2 ));
    RMSE_phi_est_2 = sqrt(mean( (rad2deg(phi2_acc(:,1))-rad2deg(phi_ref(1,:))').^2 ));

    figure(2)
    plot(t,rad2deg(theta1_acc(:,1)),t,rad2deg(theta2_acc(:,1)),t,rad2deg(theta3_acc(:,1)),t,-rad2deg(theta_ref(1,:)),LineWidth = 1.2)
    legend('theta1','theta2','theta3', 'true')
    RMSE_theta_est1 = sqrt(mean( (rad2deg(theta1_acc(:,1))+rad2deg(theta_ref(1,:))').^2 ));
    RMSE_theta_est2 = sqrt(mean( (rad2deg(theta2_acc(:,1))+rad2deg(theta_ref(1,:))').^2 ));
    RMSE_theta_est3 = sqrt(mean( (rad2deg(theta3_acc(:,1))+rad2deg(theta_ref(1,:))').^2 ));
