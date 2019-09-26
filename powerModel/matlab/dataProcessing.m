%% Load appropriate data file
clear

addpath('\\ad.uillinois.edu\engr-ews\afausti2\Desktop\BRG\OptEnd\data')

DATE = '6Aug'; % dayMonth
FLIGHT_NUM = '15';

dataFileName = strcat('flt',FLIGHT_NUM,'_',DATE,'.mat');
load(dataFileName)
data.fltNum = strcat('flt',FLIGHT_NUM,'_',DATE);

% generalize for other sections
fltData = flt15_6Aug;
data.dataPtNum = size(fltData, 1);

%% Determine velocity and radius of test 
% calculate vel norm (m/s) for every data point
velNorm = zeros(data.dataPtNum, 1);
for i=1:data.dataPtNum
    velN = fltData(i,55);
    velE = fltData(i,56);
    velD = fltData(i,57);
    velNorm(i) = norm([velN; velE; velD]);
end

data.velNorm = velNorm;
figure(2)
plot(fltData(:,2), velNorm)

% Locate beginning and ending of orbit in data
figure(1)
plot(fltData(:,2), fltData(:,45)) 
% use databrush to highlight orbit i.e. once the longitude becomes periodic
% and output a variable named traj
pause() 

% find trajectory in data
data.startIdx = find(fltData(:,2) == traj(1,1));
data.endIdx = find(fltData(:,2) == traj(end,1));

% homepoint, lat, long, and relative height to homepoint
data.home = [fltData(1,45) fltData(1,44)];
data.lat = fltData(:,45);
data.long = fltData(:,44);
data.relHeight = fltData(:,42);

% Calculate mean power at battery and standard deviation
data.power = fltData(:,77);
data.meanPower = mean(data.power);
data.stdPower = std(data.power);

% Append air speed info
data.airSpeedxin1 = fltData(:,116);
data.airSpeedyin1 = fltData(:,117);
data.airSpeedxin0 = fltData(:,120);
data.airSpeedyin0 = fltData(:,121);

%% Append current fltData to data array
% UPDATE THESE BEFORE APPENDING TO PROCESSEDFLTDATA
data.windCondition = 'medium';
data.targetLinVel = 0; % m/s
data.traj = 'orbit 25';

% gps vel data
data.velN = fltData(:,55);
data.velE = fltData(:,56);
data.velD = fltData(:,57);

% offset time
data.offsetTime = fltData(:,2);

% airspeed vel norm
data.airSpeedNorm = fltData(:,119);

% roll pitch yaw imu
data.imuRoll = fltData(:,26);
data.imuPitch = fltData(:,27);
data.imuYaw = fltData(:,28);

% imu vel data
data.imuVelN = fltData(:,20);
data.imuVelE = fltData(:,21);
data.imuVelD = fltData(:,22);

% convert to cell array
dataArr = struct2cell(data);

% append to array of all data
load('processedFltData.mat')
if isempty(processedFltData)
    processedFltData{1} = dataArr;
else
    processedFltData{end + 1} = dataArr;
end
save('processedFltData.mat', 'processedFltData')

%% Edit existing processed data file
clear

addpath('\\ad.uillinois.edu\engr-ews\afausti2\Desktop\BRG\OptEnd\data')

DATE = '18June'; % dayMonth
FLIGHT_NUM = '6';

dataFileName = strcat('flt',FLIGHT_NUM,'_',DATE,'.mat');
load(dataFileName)
data.fltNum = strcat('flt',FLIGHT_NUM,'_',DATE);

% generalize for other sections
fltData = flt6_18June;

load('processedFltData.mat')

% column of data to edit
k = 17;

processedFltData{1,k}{28,1} = fltData(:,20);
processedFltData{1,k}{29,1} = fltData(:,21);
processedFltData{1,k}{30,1} = fltData(:,22);

save('processedFltData.mat', 'processedFltData')

%% Data file index reference guide
% GPS data at 200 Hz
% IMU data at 200 Hz
% Mag data at 50 Hz
% Battery data at 1 Hz
% 2 - offset time (s)
% 8 - accel x (g)
% 9 - accel y (g) NEU coordinate system
% 10 - accel z (g)
% 11 - accel norm
% 20 - imu vel N
% 21 - imu vel E
% 22 - imu vel D
% 26 - roll (deg)
% 27 - pitch (deg)
% 28 - yaw (deg)
% 42 - Height relative to home point (m)
% 44 - GPS longitude
% 45 - GPS latitude
% 48 - GPS MSL height (m)
% 55 - GPS N velocity (m/s)
% 56 - GPS E velocity (m/s)
% 57 - GPS D veloctiy (m/s)
% 77 - power at battery (W)
% 116 - air speed body x (m/s)
% 117 - air speed body y (m/s)
% 119 - vel norm (m/s)
% 120 - air ground body x (m/s)
% 121 - air ground body y (m/s)