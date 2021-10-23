clear all
close all
clc

load('data.mat')

m = 17.85;
Fs = 1600;

Fx = - data.acc.accel_x * m; 
Fy = data.acc.accel_y * m; 
Fz = data.acc.accel_z * m; 

%% 100
tMIN = 3.8;
tMAX = 6.8;
is = min(find(data.acc.accel_timestamp >= tMIN));
ie = max(find(data.acc.accel_timestamp <= tMAX));

L = length(data.acc.accel_timestamp(is:ie));

Y=fft(Fx(is:ie));

P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);

f = Fs*(0:(L/2))/L;

figure('Name','100 aerobreaks','NumberTitle','off');
plot(f(50:end),P1(50:end))
title('100 aerobreaks')
xlabel('Frequency [Hz]')
ylabel('Force [N]')
hold on
grid on

Y=fft(Fy(is:ie));

P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);

f = Fs*(0:(L/2))/L;

plot(f(50:end),P1(50:end))

Y=fft(Fz(is:ie));

P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);

f = Fs*(0:(L/2))/L;

plot(f(50:end),P1(50:end))

legend('x','y','z')

%% 50
tMIN = 7.5;
tMAX = 9;
is = min(find(data.acc.accel_timestamp >= tMIN));
ie = max(find(data.acc.accel_timestamp <= tMAX));

L = length(data.acc.accel_timestamp(is:ie));

Y=fft(Fx(is:ie));

P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);

f = Fs*(0:(L/2))/L;

figure('Name','50 aerobreaks','NumberTitle','off');
plot(f(50:end),P1(50:end))
title('50 aerobreaks')
xlabel('Frequency [Hz]')
ylabel('Force [N]')
hold on
grid on

Y=fft(Fy(is:ie));

P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);

f = Fs*(0:(L/2))/L;

plot(f(50:end),P1(50:end))

Y=fft(Fz(is:ie));

P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);

f = Fs*(0:(L/2))/L;

plot(f(50:end),P1(50:end))

legend('x','y','z')

%% 00
tMIN = 11;
tMAX = 14;
is = min(find(data.acc.accel_timestamp >= tMIN));
ie = max(find(data.acc.accel_timestamp <= tMAX));

L = length(data.acc.accel_timestamp(is:ie));

Y=fft(Fx(is:ie));

P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);

f = Fs*(0:(L/2))/L;

figure('Name','00 aerobreaks','NumberTitle','off');
plot(f(50:end),P1(50:end))
title('00 aerobreaks')
xlabel('Frequency [Hz]')
ylabel('Force [N]')
hold on
grid on

Y=fft(Fy(is:ie));

P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);

f = Fs*(0:(L/2))/L;

plot(f(50:end),P1(50:end))

Y=fft(Fz(is:ie));

P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);

f = Fs*(0:(L/2))/L;

plot(f(50:end),P1(50:end))

legend('x','y','z')

%% burning
tMIN = 0.5;
tMAX = 2.5;
is = min(find(data.acc.accel_timestamp >= tMIN));
ie = max(find(data.acc.accel_timestamp <= tMAX));

L = length(data.acc.accel_timestamp(is:ie));

Y=fft(Fx(is:ie));

P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);

f = Fs*(0:(L/2))/L;

figure('Name','Burning','NumberTitle','off');
plot(f(50:end),P1(50:end))
title('Burning')
xlabel('Frequency [Hz]')
ylabel('Force [N]')
hold on
grid on

Y=fft(Fy(is:ie));

P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);

f = Fs*(0:(L/2))/L;

plot(f(50:end),P1(50:end))

Y=fft(Fz(is:ie));

P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);

f = Fs*(0:(L/2))/L;

plot(f(50:end),P1(50:end))

legend('x','y','z')