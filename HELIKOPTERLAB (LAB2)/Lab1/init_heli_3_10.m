% FOR HELICOPTER NR 3-10
% This file contains the initialization for the helicopter assignment in
% the course TTK4115. Run this file before you execute QuaRC_ -> Build 
% to build the file heli_q8.mdl.

% Oppdatert høsten 2006 av Jostein Bakkeheim
% Oppdatert høsten 2008 av Arnfinn Aas Eielsen
% Oppdatert høsten 2009 av Jonathan Ronen
% Updated fall 2010, Dominik Breu
% Updated fall 2013, Mark Haring
% Updated spring 2015, Mark Haring


%%%%%%%%%%% Calibration of the encoder and the hardware for the specific
%%%%%%%%%%% helicopter
Joystick_gain_x = 1;
Joystick_gain_y = -1;


%%%%%%%%%%% Physical constants
g = 9.81; % gravitational constant [m/s^2]
l_c = 0.46; % distance elevation axis to counterweight [m]
l_h = 0.66; % distance elevation axis to helicopter head [m]
l_p = 0.175; % distance pitch axis to motor [m]
m_c = 1.92; % Counterweight mass [kg]
m_p = 0.72; % Motor mass [kg]
Vs_0 = 7;
k_f = - (l_c * m_c - 2 * l_h * m_p) * g / (l_h * Vs_0);
display(k_f);
J_p = 2 * m_p * l_p * l_p;
J_e = m_c * l_c * l_c + 2 * m_p * l_h * l_h; %Lab2
L_1 = l_p * k_f;
L_3 = l_h * k_f; %Lab 2
K_1 = L_1 / J_p;
K_2 = L_3 / J_e; %Lab 2
lambda_1 = -0.8;
lambda_2 = -8;
k_pd = - (lambda_1 + lambda_2) / K_1;
k_pp = lambda_1 * lambda_2 / K_1;
q11 = 1;
q22 = 1;
q33 = 20;
q44 = 1;
q55 = 1;
r11 = 1;
r22 = 0.5;
%A = [[0 1 0];[0 0 0];[0 0 0]];
%B = [[0 0];[0 K_1];[ K_2 0]];
%C = [[1 0 0]; [0 0 1]];
%Q_lqr = [[q11 0 0];[0 q22 0];[0 0 q33]];
%R_lqr = [[r11 0];[0 r22]];
%K = lqr(A, B, Q_lqr, R_lqr);
%F = inv((C * inv(B*K - A) * B));

A_aug = [[0 1 0 0 0];[0 0 0 0 0];[0 0 0 0 0]; [-1 0 0 0 0]; [0 0 -1 0 0]];
B_aug = [[0 0];[0 K_1];[K_2 0];[0 0]; [0 0]];
C_aug = [[1 0 0 0 0]; [0 0 1 0 0]];
Q_lqr_aug = [[q11 0 0 0 0];[0 q22 0 0 0];[0 0 q33 0 0]; [0 0 0 q44 0]; [0 0 0 0 q55]];
R_lqr_aug = [[r11 0];[0 r22]];
G = [[0 0];[0 0];[0 0];[1 0]; [0 1]];
%I = [[1 0 0 0 0];[0 1 0 0 0];[0 0 1 0 0]; [0 0 0 1 0]; [0 0 0 0 1]];
I = eye(2);
K_LQR = lqr(A_aug, B_aug, Q_lqr_aug, R_lqr_aug);
k_11 = K_LQR(1, 1);
k_12 = K_LQR(1, 2);
k_13 = K_LQR(1, 3);
k_14 = K_LQR(1, 4);
k_15 = K_LQR(1, 5);
k_21 = K_LQR(2, 1);
k_22 = K_LQR(2, 2);
k_23 = K_LQR(2, 3);
k_24 = K_LQR(2, 4);
k_25 = K_LQR(2, 5);
K = [k_11 k_12 k_13; k_21 k_22 k_23];

F = inv((C * inv(B*K - A) * B));
%F_aug = inv((C_aug * inv(B_aug*K_aug - A_aug) * B_aug)) * (I - C_aug * inv(B*K - A) * G);
%F_aug = inv(B_aug) * (B_aug*K_aug - A_aug) * inv(C_aug) - inv(B_aug) * G;