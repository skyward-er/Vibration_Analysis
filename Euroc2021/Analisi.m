%% CALCOLO FREQUENZE VIBRAZIONI
% per i dati di roccaraso x e z sono inverititi, z è l asse bod verso 
clear all
close all
load('data_euroc.mat')
load('data.mat');
linecolors={'r' 'y' 'c' 'g' 'b'};
[handles]=plotNy( stack.bmx.acc.time,stack.bmx.acc.accel_x/9.8,1,...
stack.bmx_corrected.acc.time,stack.bmx_corrected.acc.accel_x/9.8,1,...
    stack.ada.time,stack.ada.msl_altitude,2,... %altitudine
    stack.brake.time,stack.brake.servo_position*2,3,...%airbrakes
    stack.pitot_calibrated.time(200:1150),stack.pitot_calibrated.airspeed(200:1150)./stack.atmosphere_data.c(200:1150),4,...
    'YAxisLabels',{ 'Accelerazione [g]' 'Altitudine [m]' 'Aerofreni [%]' 'Mach'},...
    'Linewidth',1,...
    'XLim',[-5,40],...
    'XAxisLabel','time[s]',...
    'TitleStr','Lancio euroc',...
    'FontSize',10,...
    'LineColor',linecolors,...
    'LegendString',{'Accelerazione X_{body}' 'Accelerazione mediata X_{body}' 'Altitudine (ADA)' 'Airbrakes' 'Mach'});
grid on
load('data.mat');
linecolors={'r' 'y' 'c'  'b'};
[handles]=plotNy( data.acc.accel_timestamp,data.acc.accel_z/9.8,1,...
movmean(data.acc.accel_timestamp,100),movmean(data.acc.accel_z,100)/9.8,1,...
    data.ADA.timestamp,data.ADA.msl_altitude,2,... %altitudine
    data.pitot.timestamp,data.pitot.airspeed/340,3,...
    'YAxisLabels',{ 'Accelerazione [g]' 'Altitudine [m]'  'Mach'},...
    'Linewidth',1,...
    'XLim',[-5,40],...
    'XAxisLabel','time[s]',...
    'TitleStr','Lancio Roccaraso',...
    'FontSize',10,...
    'LineColor',linecolors,...
    'LegendString',{'Accelerazione X_{body}' 'Accelerazione mediata X_{body}' 'Altitudine (ADA)'  'Mach'});
grid on
%% Analisi di rumore

Fs = 1600;
% rumore in rampa
Fz = stack.bmx.acc.accel_z;
Fy = stack.bmx.acc.accel_y; 
Fx = stack.bmx.acc.accel_x; 
tMIN=-4.5;
tMAX=-0.5;
t=stack.bmx.acc.time;
analisi_frequenze(tMIN,tMAX,t,Fz,Fx,Fy,Fs,1000,300,600,'Pre (EUROC)',2)
% Rumore 0.035
%% Analisi delle vibrazioni

Fs = 1600;
% Burning
Fz = stack.bmx.acc.accel_z;
Fy = stack.bmx.acc.accel_y; 
Fx = stack.bmx.acc.accel_x; 
tMIN=0.5;
tMAX=2.5;
t=stack.bmx.acc.time;
analisi_frequenze(tMIN,tMAX,t,Fz,Fx,Fy,Fs,500,300,800,'Burning (EUROC)',2)

Fz =data.acc.accel_x; % sono invertite
Fy = data.acc.accel_y; 
Fx = data.acc.accel_z; 
tMIN=0.5;
tMAX=2.5;
t=data.acc.accel_timestamp;
analisi_frequenze(tMIN,tMAX,t,Fz,Fx,Fy,Fs,500,300,800,'Burning (Roccaraso)',2)
%%
% Aerofreni al 100%
Fz = stack.bmx.acc.accel_z;
Fy = stack.bmx.acc.accel_y; 
Fx = stack.bmx.acc.accel_x; 
tMIN=15;
tMAX=20;
t=stack.bmx.acc.time;
analisi_frequenze(tMIN,tMAX,t,Fz,Fx,Fy,Fs,500,300,800,'Aerofreni 100% (EUROC)',2)
%%
Fz =data.acc.accel_x;
Fy = data.acc.accel_y; 
Fx = data.acc.accel_z; 
tMIN=3.8;
tMAX=6.8;
t=data.acc.accel_timestamp;
analisi_frequenze(tMIN,tMAX,t,Fz,Fx,Fy,Fs,500,300,800,'Aerofreni 100% (ROCCARASO)',2)
%%
% AEROFRENI AL 50%
Fz = stack.bmx.acc.accel_z;
Fy = stack.bmx.acc.accel_y; 
Fx = stack.bmx.acc.accel_x; 
tMIN=7;
tMAX=7.5;
t=stack.bmx.acc.time;
analisi_frequenze(tMIN,tMAX,t,Fz,Fx,Fy,Fs,500,300,800,'Aerofreni 50% (EUROC)',2)
Fz =data.acc.accel_x;
Fy = data.acc.accel_y; 
Fx = data.acc.accel_z; 
tMIN=7.5;
tMAX=9;
t=data.acc.accel_timestamp;
analisi_frequenze(tMIN,tMAX,t,Fz,Fx,Fy,Fs,500,300,800,'Aerofreni 50% (Roccaraso)',2)
% AEROFRENI AL 0%
Fz = stack.bmx.acc.accel_z;
Fy = stack.bmx.acc.accel_y; 
Fx = stack.bmx.acc.accel_x; 
tMIN=4;
tMAX=6;
t=stack.bmx.acc.time;
analisi_frequenze(tMIN,tMAX,t,Fz,Fx,Fy,Fs,500,300,800,'Aerofreni 0% (EUROC)',2)
 
Fz = data.acc.accel_x;
Fy = data.acc.accel_y; 
Fx = data.acc.accel_z; 
tMIN=11;
tMAX=14;
t=data.acc.accel_timestamp;
analisi_frequenze(tMIN,tMAX,t,Fz,Fx,Fy,Fs,500,300,800,'Aerofreni 0% (Roccaraso)',2)

function analisi_frequenze(tMIN,tMAX,t,Fz,Fx,Fy,Fs,samples,overlapped_samples,DFT_points,NOME_FASE,choiche)

is = min(find(t >= tMIN));
ie = max(find(t <= tMAX));
t=t(is:ie);

Fz_fase=Fz(is:ie);
Fx_fase=Fx(is:ie);
Fy_fase=Fy(is:ie);
if choiche==1
figure()
subplot(2,1,1);

plot(t,Fz_fase,'b');
hold on
plot(t,Fx_fase,'r');
plot(t,Fy_fase,'g');
legend('z','x','y');
xlabel('time (s)')
ylabel('Acceleration (m/s^2)')
title('Segnale nel tempo');
subplot(2,1,2);
[pxx,f] = pwelch(Fz_fase,samples,overlapped_samples,DFT_points,Fs);
plot(f(50:end),10*log10(pxx(50:end)),'b')
hold on 
[pxx,f] = pwelch(Fx_fase,samples,overlapped_samples,DFT_points,Fs);

plot(f(50:end),10*log10(pxx(50:end)),'r')
[pxx,f] = pwelch(Fy_fase,samples,overlapped_samples,DFT_points,Fs);

plot(f(50:end),10*log10(pxx(50:end)),'g')
legend('z','x','y');
xlabel('Frequency (Hz)')
ylabel('PSD (dB/Hz)')
title('Spettro');
sgtitle(NOME_FASE);
else
L = length(t);
figure()
subplot(2,1,1);

plot(t,Fz_fase,'b');
hold on
plot(t,Fx_fase,'r');
plot(t,Fy_fase,'g');
legend('z','x','y');
xlabel('time (s)')
ylabel('Acceleration (m/s^2)')
title('Segnale nel tempo');
Y=fft(Fx_fase);

P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);

f = Fs*(0:(L/2))/L;

subplot(2,1,2);
plot(f(50:end),P1(50:end))
xlabel('Frequency [Hz]')
ylabel('Acceleration [m/s^2]')
hold on
grid on

Y=fft(Fy_fase);

P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);

f = Fs*(0:(L/2))/L;

plot(f(50:end),P1(50:end))

Y=fft(Fz_fase);

P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);

f = Fs*(0:(L/2))/L;

plot(f(50:end),P1(50:end))
title('Spettro')
legend('x','y','z')
sgtitle(NOME_FASE);

end
end