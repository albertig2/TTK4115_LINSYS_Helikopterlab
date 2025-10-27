
x_flying = load('lab4_opg1_IMU_STATES_flying_50_sek.mat');
x_ro = load('lab4_opg1_IMU_STATES_I_RO_50_sek.mat');

x_ro_data = x_ro.ans';
x_flying_data = x_flying.ans';

x_ro_no_time = x_ro_data(2500:end, 2:end);
x_flying_no_time = x_flying_data(2500:end, 2:end);

cov_ro = cov(x_ro_no_time);
cov_flying = cov(x_flying_no_time);
%test = 1000*cov_flying;

Rd = cov_flying;
