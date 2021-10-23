% This script creates a .mat file containing the data of the .csv files
% It needs a folder called "logs" with telemetry data .csv file inside

clear all
close all
clc

%% GENERAL INFO
tStart = 2370.71*1e6;                                  % first istant before launch
tEnd = 2536.18*1e6;                                    % first istant before landing
N = 17;                                                % number of log
addpath('logs');
%%% name of .csv files
fileName.acc = "10BMX160Data";                         % accelerations
fileName.baroDigi = "10MS5803Data";                    % digital barometer
fileName.static = "14MPXHZ6130AData";                  % Static pressure
fileName.pitot = "N15DeathStackBoard13AirSpeedPitotE"; % Pitot
fileName.ADA = "N15DeathStackBoard7ADADataE";          % ADA
fileName.FMM = "N15DeathStackBoard9FMMStatusE";        % FMM
fileName.GPS = "12UbloxGPSData";                       % GPS
fileName.GPS = "12UbloxGPSData";                       % GPS

%% LOADING DATA
dataName = fieldnames(fileName);

for i = 1:size(dataName)
    path = sprintf("logs/log%02d_%s.csv", N, fileName.(dataName{i}));
    data.(dataName{i}) = readtable(path);
    if strcmp(fileName.(dataName{i}),"10BMX160Data")
        data.(dataName{i}) = data.(dataName{i})(30:end, :);
    end
end

%% PARSING DATA (START < t < END)
for i = 1:size(dataName)
    % get timestamp of the i-th data
    t = data.(dataName{i}){:, 1};
    
    % start/end indexes
    is = max(find(t <= tStart));
    ie = min(find(t >= tEnd));
    
    % Squeeze columns
    col = size(data.(dataName{i}), 2);
    data.(dataName{i})([1:is], :) = [];
    data.(dataName{i})([ie-is:end], :) = [];
    data.(dataName{i}){:, 1} = (data.(dataName{i}){:, 1} - tStart)./1000000;
end

%% SAVE DATA
save dataeuroc.mat 
