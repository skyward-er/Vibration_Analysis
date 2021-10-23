clear all
close all
clc

%% LOAD DATA
load('data.mat');
load('data_ascent.mat');

%% ACCELERATION
%%% Telemetry acceleration - ASCENT PHASE
tMIN = 0;
tMAX = 17.5;
is = min(find(data.acc.accel_timestamp >= tMIN));
ie = max(find(data.acc.accel_timestamp <= tMAX));
figure('Name','Acceleration - ASCENT PHASE','NumberTitle','off');
plot(data.acc.accel_timestamp(is:ie), data.acc.accel_x(is:ie));
hold on
plot(data.acc.accel_timestamp(is:ie), data.acc.accel_y(is:ie));
plot(data.acc.accel_timestamp(is:ie), data.acc.accel_z(is:ie));
grid on
xlabel('Time from T0 [s]'); ylabel('Acc [m/s^2]');
legend('x', 'y', 'z');
title('Telemetry acceleration - ASCENT PHASE');

%%% Telemetry acceleration - BURNING PHASE
tMIN = 0;
tMAX = 3.5;
is = min(find(data.acc.accel_timestamp >= tMIN));
ie = max(find(data.acc.accel_timestamp <= tMAX));
figure('Name','Acceleration - BURNING PHASE','NumberTitle','off');
plot(data.acc.accel_timestamp(is:ie), data.acc.accel_x(is:ie));
hold on
plot(data.acc.accel_timestamp(is:ie), data.acc.accel_y(is:ie));
plot(data.acc.accel_timestamp(is:ie), data.acc.accel_z(is:ie));
grid on
xlabel('Time from T0 [s]'); ylabel('Acc [m/s^2]');
legend('x', 'y', 'z');
title('Telemetry acceleration - BURNING PHASE');

%%% Telemetry acceleration - AIRBRAKING PHASE
tMIN = 3.5;
tMAX = 17.5;
is = min(find(data.acc.accel_timestamp >= tMIN));
ie = max(find(data.acc.accel_timestamp <= tMAX));
figure('Name','Acceleration - AIRBRAKING PHASE','NumberTitle','off');
plot(data.acc.accel_timestamp(is:ie), data.acc.accel_x(is:ie));
hold on
plot(data.acc.accel_timestamp(is:ie), data.acc.accel_y(is:ie));
plot(data.acc.accel_timestamp(is:ie), data.acc.accel_z(is:ie));
grid on
xlabel('Time from T0 [s]'); ylabel('Acc [m/s^2]');
legend('x', 'y', 'z');
title('Telemetry acceleration - ASCENT PHASE');

%%% COMPARISON - BURN PHASE
tMIN = 0;
tMAX = 3.5;
is = min(find(data.acc.accel_timestamp >= tMIN));
ie = max(find(data.acc.accel_timestamp <= tMAX));
figure('Name','Acceleration - COMPARISON','NumberTitle','off');
plot(data.acc.accel_timestamp(is:ie), data.acc.accel_z(is:ie));
hold on
P = polyfit(data.acc.accel_timestamp(is:ie), data.acc.accel_z(is:ie), 100);
yq = polyval(P, data.acc.accel_timestamp(is:ie));
plot(data.acc.accel_timestamp(is:ie), yq, 'LineWidth', 2);

is = min(find(data_ascent.state.T >= tMIN));
ie = max(find(data_ascent.state.T <= tMAX));
plot(data_ascent.state.T(is:ie), ...
    data_ascent.accelerations.body_acc(1,is:ie) + 9.81, 'LineWidth', 2);
grid on
xlabel('Time from T0 [s]'); ylabel('Acc [m/s^2]');
legend('telemetry', 'fit', 'simulated');
title('Telemetry acceleration - COMPARISON');

%%% COMPARISON - AIRBRAKING PHASE
tMIN = 3.3;
tMAX = 17.5;
is = min(find(data.acc.accel_timestamp >= tMIN));
ie = max(find(data.acc.accel_timestamp <= tMAX));
figure('Name','Acceleration - COMPARISON','NumberTitle','off');
plot(data.acc.accel_timestamp(is:ie), data.acc.accel_z(is:ie));
hold on
P = polyfit(data.acc.accel_timestamp(is:ie), data.acc.accel_z(is:ie), 200);
yq = polyval(P, data.acc.accel_timestamp(is:ie));
plot(data.acc.accel_timestamp(is:ie), yq, 'LineWidth', 2);

is = min(find(data_ascent.state.T >= tMIN));
ie = max(find(data_ascent.state.T <= tMAX));
plot(data_ascent.state.T(is:ie), ...
    data_ascent.accelerations.body_acc(1,is:ie) + 9.81, 'LineWidth', 2);
grid on
xlabel('Time from T0 [s]'); ylabel('Acc [m/s^2]');
legend('telemetry', 'fit', 'simulated');
title('Telemetry acceleration - COMPARISON');


%% CA
%%% COMPARISON - AIRBRAKING PHASE
tMIN = 3.3;
tMAX = 17.5;
tMIN = 3.3;
tMAX = 17.5;
is = min(find(data.acc.accel_timestamp >= tMIN));
ie = max(find(data.acc.accel_timestamp <= tMAX));
t = data.acc.accel_timestamp(is:ie);

CAtelem = giveCA(tMIN, tMAX, data);

figure('Name','CD - COMPARISON','NumberTitle','off');
plot(t, CAtelem(t));
grid on; hold on;

P = polyfit(t, CAtelem(t), 100);
yq = polyval(P, t);
plot(data.acc.accel_timestamp(is:ie), yq, 'LineWidth', 2);

is = min(find(data_ascent.state.T >= tMIN));
ie = max(find(data_ascent.state.T <= tMAX));
plot(data_ascent.state.T(is:ie), ...
    data_ascent.coeff.CA(1,is:ie), 'LineWidth', 2);

xlabel('Time from T0 [s]'); ylabel('CA [-]');
legend('telemetry', 'fit', 'simulated');
title('Telemetry CA - COMPARISON');



% %%
% figure
% plotc(acc.gyro_timestamp, rad2deg(acc.gyro_x));
% hold on
% plotc(acc.gyro_timestamp, rad2deg(acc.gyro_y));
% plotc(acc.gyro_timestamp, rad2deg(acc.gyro_z));
% grid on
% 
% figure
% 
% plotc(baro_digi.press_timestamp, baro_digi.press);
% hold on
% plotc(static.press_timestamp, static.pressure );
% grid on
% 
% figure
% plotc(ada.timestamp, ada.msl_altitude);
% hold on
% plotc(gps.gps_timestamp, gps.height);
% grid on
% 
% 
% figure
% plotc(gps.gps_timestamp, gps.velocity_down);
% hold on
% plotc(ada.timestamp, -ada.vert_speed);
% grid on
% % 
% % plot(acc.accel_timestamp, acc.accel_x);
% % hold
% % plot(acc.accel_timestamp, acc.accel_y);
% % plot(acc.accel_timestamp, acc.accel_z);
% % grid on
% % 
% % figure
% % plot(acc.gyro_timestamp, acc.gyro_x);
% % hold
% % plot(acc.gyro_timestamp, acc.gyro_y);
% % plot(acc.gyro_timestamp, acc.gyro_z);
% % grid on
% 
% figure
% plotc(pitot.timestamp, pitot.airspeed);
% grid on
% 
% figure
% plotc(static.press_timestamp, atmospalt(static.pressure)-1090);
% grid on

function [CA] = giveCA(tMIN, tMAX, data)

S = pi*(0.15^2)/4;
m = 17.85;

% Velocity
is = min(find(data.pitot.timestamp >= tMIN));
ie = max(find(data.pitot.timestamp <= tMAX));
tV = data.pitot.timestamp(is:ie);
V = @(t) spline(tV, data.pitot.airspeed(is:ie), t);

% Height
is = min(find(data.GPS.gps_timestamp >= tMIN));
ie = max(find(data.GPS.gps_timestamp <= tMAX));
th = data.GPS.gps_timestamp(is:ie);
H = @(t) spline(th, data.GPS.height(is:ie), t);

% Density
[~, ~, ~, rho] = atmosisa(data.GPS.height(is:ie));
RHO = @(t) spline(th, rho, t);

% Acceleration
is = min(find(data.acc.accel_timestamp >= tMIN));
ie = max(find(data.acc.accel_timestamp <= tMAX));
tACC = data.acc.accel_timestamp(is:ie);
ACC = @(t) spline(tACC, data.acc.accel_z(is:ie), t);


CA = @(t) (2*m*(-ACC(t)))./(RHO(t) .* ((V(t)).^2) * S);

end