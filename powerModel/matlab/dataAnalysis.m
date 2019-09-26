%% Load aggregate data file and quad specs
clear

load('processedFltData.mat')

%% Find power efficiency from hover data
hoverPow = [processedFltData{1,5}{10,1}(processedFltData{1,5}{4,1}:200:processedFltData{1,5}{5,1});
            processedFltData{1,10}{10,1}(processedFltData{1,10}{4,1}:200:processedFltData{1,10}{5,1});
            processedFltData{1,15}{10,1}(processedFltData{1,15}{4,1}:200:processedFltData{1,15}{5,1});
            processedFltData{1,20}{10,1}(processedFltData{1,20}{4,1}:200:processedFltData{1,20}{5,1});
            processedFltData{1,25}{10,1}(processedFltData{1,25}{4,1}:200:processedFltData{1,25}{5,1});
            processedFltData{1,30}{10,1}(processedFltData{1,30}{4,1}:200:processedFltData{1,30}{5,1});];
        
meanHoverPow = mean(hoverPow);
%% quadrotor and environment specs
params.m = 3.29; % mass (kg)
params.R = convlength(13.5/2, 'in', 'm'); % rotor radius
params.A = 4*pi*params.R^2; % rotor swept area in m^2
params.rho = 1.225; % sea level density in kg/m^3
params.g = 9.80665; % gravitational acceleration in m/s^2
params.mu1 = 0.1816; % linear drag coefficient
params.mu2 = -0.02326; % quadratic drag coefficient
params.mu3 = 0.004045; % cubic drag coefficient
params.qmu1 = -1.219;
params.qmu2 = 0.1129;
params.v_h = sqrt(params.m*params.g/2/params.rho/pi/params.R^2); % induced velocity at hover
params.k = 0.2750; % profile power constant from liu
params.eta = meanHoverPow / (params.m*params.g*params.v_h); % efficiency

%% Average Wind magnitude and heading for each experiment
% number of experiments
N = 30;

windMagHead = zeros(N,2);
%windMagHead2 = zeros(N,2);
v_Win0Mean = zeros(N,2);
for exp=1:N
    startIdx = processedFltData{1,exp}{4,1};
    endIdx = processedFltData{1,exp}{5,1};
    windSpeedx = processedFltData{1, exp}{13, 1};
    windSpeedy = processedFltData{1, exp}{14, 1};
    
    %v_Win0Mean(exp,1) = mean(windSpeedx);
    %v_Win0Mean(exp,2) = mean(windSpeedy);
    
    %windMagHead2(exp,1) = sqrt(v_Win0Mean(exp,1)^2+v_Win0Mean(exp,2)^2);
    %windMagHead2(exp,2) = atan2(v_Win0Mean(exp,2), v_Win0Mean(exp,1));
    
    v_Win0 = [(windSpeedx)'; (windSpeedy)'];
    v_Wnorm = zeros(size(v_Win0,2), 1);
    for i=1:length(v_Wnorm)
        v_Wnorm(i) = sqrt(v_Win0(1,i)^2 + v_Win0(2,i)^2);
    end
    
    % wind heading
    windHead = atan2(v_Win0(2,:), v_Win0(1,:));
    
    windMagHead(exp,1) = mean(v_Wnorm(startIdx:endIdx));
    windMagHead(exp,2) = mean(windHead(startIdx:endIdx));
end

%% Convert to radians and match to instantaneous power measurement

% Number of flights
N = 30;

orbitPositions = cell(1, N);
powAll = cell(1, N);

for i=1:N
    [orbitPositions{1, i}, powAll{1, i}] = getPowerVsOrbitPos(processedFltData{1, i}, windMagHead(i,2)); 
end

%% Complete v_inf vector in world frame
% assume level flight and planar wind so z component is always 0
% Number of flights
N = 30;

allv_inf = cell(1, N);

for i=1:N
    allv_inf{1, i} = getv_inf(processedFltData{1, i}); 
end

%% Estimate power for all experiment
% Number of flights
N = 30;

% each cell of power data is a 3 column matrix where the first column is
% our power estimate, the second is the model we compare against, and the
% third is the measured power
allPowerData = cell(1,N);

for k=1:N
    startIdx = processedFltData{1,k}{4,1};
    endIdx = processedFltData{1,k}{5,1};
    roll = deg2rad(processedFltData{1,k}{25,1}(startIdx:40:endIdx));
    pitch = deg2rad(processedFltData{1,k}{26,1}(startIdx:40:endIdx));
    yaw = deg2rad(processedFltData{1,k}{27,1}(startIdx:40:endIdx));
    vz_in0 = processedFltData{1,k}{30,1}(startIdx:40:endIdx);
    v_inf = processedFltData{1, k}{24,1}(startIdx:40:endIdx);

    powEstFull = zeros(size(roll));
    powEstNoPara = zeros(size(roll));
    powEstNoProf = zeros(size(roll));
    powEstNoInd = zeros(size(roll));
    powEstNoClimb = zeros(size(roll));
    for i=1:length(powEstFull)
        [powEstFull(i),...
         powEstNoPara(i),...
         powEstNoProf(i),...
         powEstNoInd(i),...
         powEstNoClimb(i)] = estimatePower(v_inf(i,:),...
                                  vz_in0(i),...
                                  roll(i),...
                                  pitch(i),...
                                  params);
    end

    powEstFull = powEstFull(1:5:end);
    powEstNoPara = powEstNoPara(1:5:end);
    powEstNoProf = powEstNoProf(1:5:end);
    powEstNoInd = powEstNoInd(1:5:end);
    powEstNoClimb = powEstNoClimb(1:5:end);
    pow = processedFltData{1,k}{10,1}(startIdx:200:endIdx);
    allPowerData{1,k} = [powEstFull,...
                         powEstNoPara,...
                         powEstNoProf,...
                         powEstNoInd,...
                         powEstNoClimb,...
                         pow];
end

%% Compare power models to measured power in a single experiment
exp = 4;
startIdx = processedFltData{1,exp}{4,1};
endIdx = processedFltData{1,exp}{5,1};

t0 = processedFltData{1,exp}{23,1}(startIdx);
t = processedFltData{1,exp}{23,1}(startIdx:200:endIdx) - t0;

powrmse = sqrt(mean((allPowerData{1,exp}(:,6) - allPowerData{1,exp}(:,1)).^2));
powrmseNoPara = sqrt(mean((allPowerData{1,exp}(:,6) - allPowerData{1,exp}(:,2)).^2));
powrmseNoProf = sqrt(mean((allPowerData{1,exp}(:,6) - allPowerData{1,exp}(:,3)).^2));
powrmseNoInd = sqrt(mean((allPowerData{1,exp}(:,6) - allPowerData{1,exp}(:,4)).^2));
powrmseNoClimb = sqrt(mean((allPowerData{1,exp}(:,6) - allPowerData{1,exp}(:,5)).^2));

% fullStr = 'full model RMSE: %.2f';
% noParaStr = 'no parasitic power RMSE: %.2f';
% noProfStr = 'no profile power RMSE: %.2f';
% noIndStr = 'no induced power RMSE: %.2f';
% noClimbStr = 'no climbing power RMSE: %.2f';

figure(3)
%subplot(2,2,1)
plot(t, allPowerData{1,exp}(:,1), '-b',...
     t, allPowerData{1,exp}(:,2), '-g',...
     t, allPowerData{1,exp}(:,3), '-r',...
     t, allPowerData{1,exp}(:,4), '-m',...
     t, allPowerData{1,exp}(:,5), '-y',...
     t, allPowerData{1,exp}(:,6), '-k',...
     'LineWidth', 2);
xlabel('Time (s)')
ylabel('Instantaneous power (W)')
legend({'full model',...
       'no para model',...
       'no prof model',...
       'no Ind model',...
       'no climb model',...
       'measured power'},...
       'Location', 'northwest',...
       'Interpreter', 'latex')
set(gca, 'fontsize', 18);
axis([0 Inf -Inf Inf])

powFullGradient =  gradient(allPowerData{1,exp}(:,1),200);
powNoParaGradient =  gradient(allPowerData{1,exp}(:,2),200);
powNoProfGradient =  gradient(allPowerData{1,exp}(:,3),200);
powNoIndGradient =  gradient(allPowerData{1,exp}(:,4),200);
powNoClimbGradient =  gradient(allPowerData{1,exp}(:,5),200);
powGradient =  gradient(allPowerData{1,exp}(:,6),200);

subplot(2,1,1)
plot(t, powFullGradient, '-b',...     
     t, powNoParaGradient, '-g',...
     t, powNoProfGradient, '-r',...
     t, powNoIndGradient, '-m',...
     t, powNoClimbGradient, '-y',...
     t, powGradient, '-k',...
     'LineWidth', 2);
xlabel('Time (s)')
ylabel('Gradient (W/s)')
legend({'full model',...
       'no para model',...
       'no prof model',...
       'no Ind model',...
       'no climb model',...
       'measured power'},...
       'Location', 'northwest',...
       'Interpreter', 'latex')
set(gca, 'fontsize', 18);
axis([0 Inf -Inf Inf])

subplot(2,1,2)
plot(t, powFullGradient - powGradient, '-b',...     
     t, powNoParaGradient - powGradient, '-g',...
     t, powNoProfGradient - powGradient, '-r',...
     t, powNoIndGradient - powGradient, '-m',...
     t, powNoClimbGradient - powGradient, '-y',...
     'LineWidth', 2);
xlabel('Time (s)')
ylabel('Gradient Absolute Error (W/s)')
legend({'full model',...
       'no para model',...
       'no prof model',...
       'no Ind model',...
       'no climb model'},...
       'Location', 'northwest',...
       'Interpreter', 'latex')
set(gca, 'fontsize', 18);
axis([0 Inf -Inf Inf])

%% Make table of rmse values
fullRmse = zeros(30,1);
noParaRmse = zeros(30,1);
noProfRmse = zeros(30,1);
noIndRmse = zeros(30,1);
noClimbRmse = zeros(30,1); 
 
N = 30;
 
for exp=1:N
    fullRmse(exp,1) = sqrt(mean((allPowerData{1,exp}(:,6) - allPowerData{1,exp}(:,1)).^2));
    noParaRmse(exp,1) = sqrt(mean((allPowerData{1,exp}(:,6) - allPowerData{1,exp}(:,2)).^2));
    noProfRmse(exp,1) = sqrt(mean((allPowerData{1,exp}(:,6) - allPowerData{1,exp}(:,3)).^2));
    noIndRmse(exp,1) = sqrt(mean((allPowerData{1,exp}(:,6) - allPowerData{1,exp}(:,4)).^2));
    noClimbRmse(exp,1) = sqrt(mean((allPowerData{1,exp}(:,6) - allPowerData{1,exp}(:,5)).^2));
end

% separates the rmse table by velocity. 1st row is hover (0), 2nd is 2, 3rd is 4, 4th is 6, and 5th is 8
fullRmseMean = mean(fullRmse);
noParaRmseMean = mean(noParaRmse);
noProfRmseMean = mean(noProfRmse);
noIndRmseMean = mean(noIndRmse);
noClimbRmseMean = mean(noClimbRmse);

fullRmseWithVel = cell(5,1);
noParaRmseWithVel = cell(5,1);
noProfRmseWithVel = cell(5,1);
noIndRmseWithVel = cell(5,1);
noClimbRmseWithVel = cell(5,1);
 
for i=1:N
    v = processedFltData{1,i}{18,1};
    switch v
        case 0
            k = 1;
        case 2
            k = 2;
        case 4
            k = 3;
        case 6
            k = 4;
        case 8
            k = 5;
    end
    fullRmseWithVel{k,1} = [fullRmseWithVel{k,1} fullRmse(i,1)];
    noParaRmseWithVel{k,1} = [noParaRmseWithVel{k,1} noParaRmse(i,1)];
    noProfRmseWithVel{k,1} = [noProfRmseWithVel{k,1} noProfRmse(i,1)];
    noIndRmseWithVel{k,1} = [noIndRmseWithVel{k,1} noIndRmse(i,1)];
    noClimbRmseWithVel{k,1} = [noClimbRmseWithVel{k,1} noClimbRmse(i,1)];
end

%fullRmseMeanWithVel = zeros(5,1);
%noParaRmseMeanWithVel = zeros(5,1);
%noProfRmseMeanWithVel = zeros(5,1);
%noIndRmseMeanWithVel = zeros(5,1);
%noClimbRmseMeanWithVel = zeros(5,1); 
rmseMeanWithVel = zeros(5,6);
rmseMeanWithVel(1,1) = 0;
rmseMeanWithVel(2,1) = 2;
rmseMeanWithVel(3,1) = 4;
rmseMeanWithVel(4,1) = 6;
rmseMeanWithVel(5,1) = 8;

for i=1:5
    %fullRmseMeanWithVel(i,1) = mean(fullRmseWithVel{i,1});
    %noParaRmseMeanWithVel(i,1) = mean(noParaRmseWithVel{i,1});
    %noProfRmseMeanWithVel(i,1) = mean(noProfRmseWithVel{i,1});
    %noIndRmseMeanWithVel(i,1) = mean(noIndRmseWithVel{i,1});
    %noClimbRmseMeanWithVel(i,1) = mean(noClimbRmseWithVel{i,1});  
    rmseMeanWithVel(i,2) = mean(fullRmseWithVel{i,1});
    rmseMeanWithVel(i,3) = mean(noParaRmseWithVel{i,1});
    rmseMeanWithVel(i,4) = mean(noProfRmseWithVel{i,1});
    rmseMeanWithVel(i,5) = mean(noIndRmseWithVel{i,1});
    rmseMeanWithVel(i,6) = mean(noClimbRmseWithVel{i,1}); 
end

%% Make table of gradient rmse values, separated by velocity
N = 30; % number of experiments

fullGradRmse = zeros(N,1);
noParaGradRmse = zeros(N,1);
noProfGradRmse = zeros(N,1);
noIndGradRmse = zeros(N,1);
noClimbGradRmse = zeros(N,1);
for i = 1:N
    powFullGrad =  gradient(allPowerData{1,i}(:,1),200);
    powNoParaGrad =  gradient(allPowerData{1,i}(:,2),200);
    powNoProfGrad =  gradient(allPowerData{1,i}(:,3),200);
    powNoIndGrad =  gradient(allPowerData{1,i}(:,4),200);
    powNoClimbGrad =  gradient(allPowerData{1,i}(:,5),200);
    powGrad =  gradient(allPowerData{1,i}(:,6),200);
    
    fullGradRmse(i,1) = sqrt(mean((powFullGrad-powGrad).^2));
    noParaGradRmse(i,1) = sqrt(mean((powNoParaGrad-powGrad).^2));
    noProfGradRmse(i,1) = sqrt(mean((powNoProfGrad-powGrad).^2));
    noIndGradRmse(i,1) = sqrt(mean((powNoIndGrad-powGrad).^2));
    noClimbGradRmse(i,1) = sqrt(mean((powNoClimbGrad-powGrad).^2));
end

fullGradRmseVel = cell(5,1);
noParaGradRmseVel = cell(5,1);
noProfGradRmseVel = cell(5,1);
noIndGradRmseVel = cell(5,1);
noClimbGradRmseVel = cell(5,1);
for i = 1:N

    v = processedFltData{1,i}{18,1};
    switch v
        case 0
            k = 1;
        case 2
            k = 2;
        case 4
            k = 3;
        case 6
            k = 4;
        case 8
            k = 5;
    end
    
    fullGradRmseVel{k,1} = [fullGradRmseVel{k,1} fullGradRmse(i,1)];
    noParaGradRmseVel{k,1} = [noParaGradRmseVel{k,1} noParaGradRmse(i,1)];
    noProfGradRmseVel{k,1} = [noProfGradRmseVel{k,1} noProfGradRmse(i,1)];
    noIndGradRmseVel{k,1} = [noIndGradRmseVel{k,1} noIndGradRmse(i,1)];
    noClimbGradRmseVel{k,1} = [noClimbGradRmseVel{k,1} noClimbGradRmse(i,1)];
end

rmseMeanGrad = zeros(5,6);
rmseMeanGrad(1,1) = 0;
rmseMeanGrad(2,1) = 2;
rmseMeanGrad(3,1) = 4;
rmseMeanGrad(4,1) = 6;
rmseMeanGrad(5,1) = 8;

for i=1:5
    %fullRmseMeanGrad(i,1) = mean(fullRmseWithVel{i,1});
    %noParaRmseMeanWithVel(i,1) = mean(noParaRmseWithVel{i,1});
    %noProfRmseMeanWithVel(i,1) = mean(noProfRmseWithVel{i,1});
    %noIndRmseMeanWithVel(i,1) = mean(noIndRmseWithVel{i,1});
    %noClimbRmseMeanWithVel(i,1) = mean(noClimbRmseWithVel{i,1});  
    rmseMeanGrad(i,2) = mean(fullGradRmseVel{i,1});
    rmseMeanGrad(i,3) = mean(noParaGradRmseVel{i,1});
    rmseMeanGrad(i,4) = mean(noProfGradRmseVel{i,1});
    rmseMeanGrad(i,5) = mean(noIndGradRmseVel{i,1});
    rmseMeanGrad(i,6) = mean(noClimbGradRmseVel{i,1}); 
end

%% Create matrix with individual orbits
% structure of matrix:
% cell row | cell column | start index | end index
% each row should be a singular orbit
N = 30;
%exp = 2;

orbits2mps = [];
orbits4mps = [];
orbits6mps = [];
orbits8mps = [];

for exp=1:N
  
    v = processedFltData{1,exp}{18,1};
    
    if v ~= 0    
        orbitsTemp = [];
        index = 1;
        while orbitPositions{1,exp}(index,1)<orbitPositions{1,exp}(index+1,1)
            index = index + 1;
        end
        index = index+1;
        nextStartIdx = index;
        for i=index:length(orbitPositions{1,exp})-1
            if orbitPositions{1,exp}(i,1)>orbitPositions{1,exp}(i+1,1)
                orbitsTemp = [orbitsTemp;1 exp nextStartIdx i];
                nextStartIdx = i+1;
            end
        end
    end
    
    switch v
        case 0
            k = 1;
        case 2
            orbits2mps = [orbits2mps; orbitsTemp];
        case 4
            orbits4mps = [orbits4mps; orbitsTemp];
        case 6
            orbits6mps = [orbits6mps; orbitsTemp];
        case 8
            orbits8mps = [orbits8mps; orbitsTemp];
    end
end

%% Find windowed average power for individual orbits
% structure of matrix:
% cell row | cell column | start index | end index
% each row should be a singular orbit

% orbits2mps = [1 1 112 183;
%               1 1 184 254;
%               1 6 36 105;
%               1 6 106 173;
%               1 11 37 107;
%               1 16 15 157;
%               1 21 1 145;
%               1 26 1 145];
% orbits4mps = [1 2 12 44;
%               1 2 45 78;
%               1 2 79 112;
%               1 2 113 145;
%               1 2 146 179;
%               1 2 180 212;
%               1 7 13 45;
%               1 7 46 78;
%               1 7 79 111;
%               1 7 112 145;
%               1 7 146 178;
%               1 12 11 44;
%               1 12 45 77;
%               1 12 78 111;
%               1 17 43 113;
%               1 22 39 110;
%               1 27 34 103];
% orbits6mps = [1 3 7 29;
%               1 3 30 51;
%               1 3 52 74;
%               1 3 75 96;
%               1 3 97 118;
%               1 3 119 141;
%               1 3 142 163;
%               1 3 164 185;
%               1 3 186 208;
%               1 8 4 26;
%               1 8 27 48;
%               1 8 49 71;
%               1 8 72 93;
%               1 8 94 116;
%               1 8 117 138;
%               1 8 139 161;
%               1 8 162 183;
%               1 8 184 205;
%               1 13 4 26;
%               1 13 27 48;
%               1 13 49 70;
%               1 13 71 93;
%               1 13 94 115;
%               1 13 116 138;
%               1 18 17 61;
%               1 18 62 107;
%               1 18 108 152;
%               1 23 14 60;
%               1 23 61 107;
%               1 28 17 62;
%               1 28 63 108];
% orbits8mps = [1 4 5 22;
%               1 4 23 39;
%               1 4 40 56;
%               1 4 57 74;
%               1 4 75 91;
%               1 4 92 108;
%               1 4 109 126;
%               1 4 127 143;
%               1 4 144 161;
%               1 4 162 178;
%               1 4 179 196;
%               1 4 197 213;
%               1 4 214 231;
%               1 9 17 33;
%               1 9 34 50;
%               1 9 51 67;
%               1 9 68 84;
%               1 9 85 101;
%               1 9 102 118
%               1 9 119 135;
%               1 9 136 152;
%               1 9 153 169;
%               1 9 170 186;
%               1 9 187 203;
%               1 9 204 220;
%               1 14 16 32;
%               1 14 33 48;
%               1 14 49 65;
%               1 14 66 82;
%               1 14 83 98;
%               1 14 99 115;
%               1 19 9 43;
%               1 19 44 78;
%               1 19 79 113;
%               1 24 17 51;
%               1 24 52 86;
%               1 24 87 121;
%               1 29 13 47;
%               1 29 48 82;
%               1 29 83 116];

orbits2mps = [orbits2mps;
              1 21 1 145;
              1 26 1 145];

orbitsToAnalyze = [orbits2mps; orbits4mps; orbits6mps; orbits8mps];
% orbitsToAnalyze = orbits8mps;

% window size
s = 0.5;
% radian resolution
r = 30;

powAvg = zeros(r, size(orbitsToAnalyze, 1));
powEstAvg = zeros(r, size(orbitsToAnalyze, 1));
powEstNoParaAvg = zeros(r, size(orbitsToAnalyze, 1));
powEstNoProfAvg = zeros(r, size(orbitsToAnalyze, 1));
powEstNoIndAvg = zeros(r, size(orbitsToAnalyze, 1));
powEstNoClimbAvg = zeros(r, size(orbitsToAnalyze, 1));
for i =1:size(orbitsToAnalyze, 1)
    a = orbitsToAnalyze(i, 1);
    b = orbitsToAnalyze(i, 2);
    c = orbitsToAnalyze(i, 3);
    d = orbitsToAnalyze(i, 4);
    
    powAvg(:, i) = getAvgPowOfRad(flip(orbitPositions{a,b}(c:d)),...
                    flip(powAll{a,b}(c:d)), s, r);
                
    powEstAvg(:, i) = getAvgPowOfRad(flip(orbitPositions{a,b}(c:d)),...
                      flip(allPowerData{a,b}(c:d,1)), s, r);
                  
    powEstNoParaAvg(:, i) = getAvgPowOfRad(flip(orbitPositions{a,b}(c:d)),...
                        flip(allPowerData{a,b}(c:d,2)), s, r);
                    
    powEstNoProfAvg(:, i) = getAvgPowOfRad(flip(orbitPositions{a,b}(c:d)),...
                        flip(allPowerData{a,b}(c:d,3)), s, r);
                    
    powEstNoIndAvg(:, i) = getAvgPowOfRad(flip(orbitPositions{a,b}(c:d)),...
                        flip(allPowerData{a,b}(c:d,4)), s, r);
                    
    powEstNoClimbAvg(:, i) = getAvgPowOfRad(flip(orbitPositions{a,b}(c:d)),...
                        flip(allPowerData{a,b}(c:d,5)), s, r);
end

powRollMean = mean(powAvg, 2);
powEstRollMean = mean(powEstAvg, 2);
powEstNoParaRollMean = mean(powEstNoParaAvg, 2);
powEstNoProfRollMean = mean(powEstNoProfAvg, 2);
powEstNoIndRollMean = mean(powEstNoIndAvg, 2);
powEstNoClimbRollMean = mean(powEstNoClimbAvg, 2);

figure(1)
rad = linspace(0, 2*pi, r);
p2 = plot(rad, powRollMean, '-k',...
     rad, powEstRollMean, '-b',...
     rad, powEstNoParaRollMean, '-g',...
     rad, powEstNoProfRollMean, '-r',...
     rad, powEstNoIndRollMean, '-m',...
     rad, powEstNoClimbRollMean, '-y',...
     'LineWidth', 2);
xlabel('Position in orbit (rad)')
ylabel('Instantaneous power (W)')
legend(p2, {'mean $P$ over 24 orbits',...
       'mean $P_{est}$ full model',...
       'mean $P_{est}$ no parasitic',...
       'mean $P_{est}$ no profile',...
       'mean $P_{est}$ no induced',...
       'mean $P_{est}$ no climb'},...
       'Location', 'east',...
       'Interpreter', 'latex')
set(gca, 'fontsize', 18);
axis([0 Inf -Inf Inf])

for j =1:4
    switch j
        case 1
            orbitsToAnalyze = orbits2mps;
        case 2
            orbitsToAnalyze = orbits4mps;
        case 3
            orbitsToAnalyze = orbits6mps;
        case 4
            orbitsToAnalyze = orbits8mps;
    end
    
    powAvg = zeros(r, size(orbitsToAnalyze, 1));
    powEstAvg = zeros(r, size(orbitsToAnalyze, 1));
    powEstNoParaAvg = zeros(r, size(orbitsToAnalyze, 1));
    powEstNoProfAvg = zeros(r, size(orbitsToAnalyze, 1));
    powEstNoIndAvg = zeros(r, size(orbitsToAnalyze, 1));
    powEstNoClimbAvg = zeros(r, size(orbitsToAnalyze, 1));
    for i =1:size(orbitsToAnalyze)
        a = orbitsToAnalyze(i, 1);
        b = orbitsToAnalyze(i, 2);
        c = orbitsToAnalyze(i, 3);
        d = orbitsToAnalyze(i, 4);
        
        powAvg(:, i) = getAvgPowOfRad(flip(orbitPositions{a,b}(c:d)),...
            flip(powAll{a,b}(c:d)), s, r);
        
        powEstAvg(:, i) = getAvgPowOfRad(flip(orbitPositions{a,b}(c:d)),...
            flip(allPowerData{a,b}(c:d,1)), s, r);
        
        powEstNoParaAvg(:, i) = getAvgPowOfRad(flip(orbitPositions{a,b}(c:d)),...
            flip(allPowerData{a,b}(c:d,2)), s, r);
        
        powEstNoProfAvg(:, i) = getAvgPowOfRad(flip(orbitPositions{a,b}(c:d)),...
            flip(allPowerData{a,b}(c:d,3)), s, r);
        
        powEstNoIndAvg(:, i) = getAvgPowOfRad(flip(orbitPositions{a,b}(c:d)),...
            flip(allPowerData{a,b}(c:d,4)), s, r);
        
        powEstNoClimbAvg(:, i) = getAvgPowOfRad(flip(orbitPositions{a,b}(c:d)),...
            flip(allPowerData{a,b}(c:d,5)), s, r);
    end
    
    powRollMean = mean(powAvg, 2);
    powEstRollMean = mean(powEstAvg, 2);
    powEstNoParaRollMean = mean(powEstNoParaAvg, 2);
    powEstNoProfRollMean = mean(powEstNoProfAvg, 2);
    powEstNoIndRollMean = mean(powEstNoIndAvg, 2);
    powEstNoClimbRollMean = mean(powEstNoClimbAvg, 2);
    
    figure(10)
    subplot(2,2,j)
    p1 = plot(rad, abs(powEstRollMean - powRollMean), '-b',...
        rad, abs(powEstNoParaRollMean - powRollMean), '-g',...
        rad, abs(powEstNoProfRollMean - powRollMean), '-r',...
        rad, abs(powEstNoIndRollMean - powRollMean), '-m',...
        rad, abs(powEstNoClimbRollMean - powRollMean), '-y',...
        'LineWidth', 2);
    title([num2str(2*j),'mps orbit']);
    xlabel('Position in orbit (rad)')
    ylabel('Instantaneous power absolute error (W)')
    legend(p1, {
        'mean $P_{est}$ full model',...
        'mean $P_{est}$ no parasitic',...
        'mean $P_{est}$ no profile',...
        'mean $P_{est}$ no induced',...
        'mean $P_{est}$ no climb'},...
        'Location', 'east',...
        'Interpreter', 'latex')
    set(gca, 'fontsize', 12);
    axis([0 2*pi -Inf Inf])
    
    figure(18)
    subplot(2,4,j)
    p2 = plot(rad, abs(powEstRollMean - powRollMean)./powRollMean*100, '-b',...
        rad, abs(powEstNoParaRollMean - powRollMean)./powRollMean*100, '-g',...
        rad, abs(powEstNoProfRollMean - powRollMean)./powRollMean*100, '-r',...
        rad, abs(powEstNoIndRollMean - powRollMean)./powRollMean*100, '-m',...
        rad, abs(powEstNoClimbRollMean - powRollMean)./powRollMean*100, '-y',...
        'LineWidth', 2);
    title([num2str(2*j),'mps orbit']);
    xlabel('Position in orbit (rad)')
    ylabel('Instantaneous power relative error (%)')
    legend(p2, {
        'mean $P_{est}$ full model',...
        'mean $P_{est}$ no parasitic',...
        'mean $P_{est}$ no profile',...
        'mean $P_{est}$ no induced',...
        'mean $P_{est}$ no climb'},...
        'Location', 'east',...
        'Interpreter', 'latex')
    set(gca, 'fontsize', 12);
    axis([0 2*pi -Inf Inf])
end

%% Plot as gradient error

orbitsToAnalyze = [orbits2mps; orbits4mps; orbits6mps; orbits8mps];
% orbitsToAnalyze = orbits8mps;

% window size
s = 0.5;
% radian resolution
r = 30;
rad = linspace(0, 2*pi, r);
spacing = 2*pi/(r-1);

xAxis = zeros(size(rad));

GradRadRmse = zeros(4,5);
for j =1:4
    switch j
        case 1
            orbitsToAnalyze = orbits2mps;
        case 2
            orbitsToAnalyze = orbits4mps;
        case 3
            orbitsToAnalyze = orbits6mps;
        case 4
            orbitsToAnalyze = orbits8mps;
    end
    
    powAvg = zeros(r, size(orbitsToAnalyze, 1));
    powEstAvg = zeros(r, size(orbitsToAnalyze, 1));
    powEstNoParaAvg = zeros(r, size(orbitsToAnalyze, 1));
    powEstNoProfAvg = zeros(r, size(orbitsToAnalyze, 1));
    powEstNoIndAvg = zeros(r, size(orbitsToAnalyze, 1));
    powEstNoClimbAvg = zeros(r, size(orbitsToAnalyze, 1));
    for i =1:size(orbitsToAnalyze)
        a = orbitsToAnalyze(i, 1);
        b = orbitsToAnalyze(i, 2);
        c = orbitsToAnalyze(i, 3);
        d = orbitsToAnalyze(i, 4);
        
        powAvg(:, i) = getAvgPowOfRad(flip(orbitPositions{a,b}(c:d)),...
            flip(powAll{a,b}(c:d)), s, r);
        
        powEstAvg(:, i) = getAvgPowOfRad(flip(orbitPositions{a,b}(c:d)),...
            flip(allPowerData{a,b}(c:d,1)), s, r);
        
        powEstNoParaAvg(:, i) = getAvgPowOfRad(flip(orbitPositions{a,b}(c:d)),...
            flip(allPowerData{a,b}(c:d,2)), s, r);
        
        powEstNoProfAvg(:, i) = getAvgPowOfRad(flip(orbitPositions{a,b}(c:d)),...
            flip(allPowerData{a,b}(c:d,3)), s, r);
        
        powEstNoIndAvg(:, i) = getAvgPowOfRad(flip(orbitPositions{a,b}(c:d)),...
            flip(allPowerData{a,b}(c:d,4)), s, r);
        
        powEstNoClimbAvg(:, i) = getAvgPowOfRad(flip(orbitPositions{a,b}(c:d)),...
            flip(allPowerData{a,b}(c:d,5)), s, r);
    end
   
    powRollMean = mean(powAvg, 2);
    powEstRollMean = mean(powEstAvg, 2);
    powEstNoParaRollMean = mean(powEstNoParaAvg, 2);
    powEstNoProfRollMean = mean(powEstNoProfAvg, 2);
    powEstNoIndRollMean = mean(powEstNoIndAvg, 2);
    powEstNoClimbRollMean = mean(powEstNoClimbAvg, 2);
    
    powGrad = gradient(powRollMean,spacing);
    powEstGrad = gradient(powEstRollMean,spacing);
    powEstNoParaGrad = gradient(powEstNoParaRollMean,spacing);
    powEstNoProfGrad = gradient(powEstNoProfRollMean,spacing);
    powEstNoIndGrad = gradient(powEstNoIndRollMean,spacing);
    powEstNoClimbGrad = gradient(powEstNoClimbRollMean,spacing);
    
    figure(18)
    subplot(2,4,j+4)
    
    p1 = plot(rad, powGrad, '-k',...
        rad, powEstGrad, '-b',...
        rad, powEstNoParaGrad, '-g',...
        rad, powEstNoProfGrad, '-r',...
        rad, powEstNoIndGrad, '-m',...
        rad, powEstNoClimbGrad, '-y',...
        rad, xAxis,'--',...
        'LineWidth', 2);
    title([num2str(2*j),'mps orbit'])
    xlabel('Position in orbit (rad)')
    ylabel('Gradient (W/rad)')
    legend(p1, {
        'mean $P$ over all orbits', ...
        'mean $P_{est}$ full model',...
        'mean $P_{est}$ no parasitic',...
        'mean $P_{est}$ no profile',...
        'mean $P_{est}$ no induced',...
        'mean $P_{est}$ no climb'},...
        'Location', 'southeast',...
        'Interpreter', 'latex')    
    set(gca, 'fontsize', 18);
    axis([0 2*pi -Inf Inf])
    % drawaxis(gca, 'x', 0, 'movelabel', 1)
    
    figure(14)
    subplot(2,2,j)
    p2 = plot(rad, (powEstGrad - powGrad), '-b',...
        rad, (powEstNoParaGrad - powGrad), '-g',...
        rad, (powEstNoProfGrad - powGrad), '-r',...
        rad, (powEstNoIndGrad - powGrad), '-m',...
        rad, (powEstNoClimbGrad - powGrad), '-y',...
        'LineWidth', 2);
    
    title([num2str(2*j),'mps orbit'])
    xlabel('Position in orbit (rad)')
    ylabel('Gradient Absolute Error (W/rad)')
    legend(p2, {
        'mean $P_{est}$ full model',...
        'mean $P_{est}$ no parasitic',...
        'mean $P_{est}$ no profile',...
        'mean $P_{est}$ no induced',...
        'mean $P_{est}$ no climb'},...
        'Location', 'southeast',...
        'Interpreter', 'latex')    
    set(gca, 'fontsize', 18);
    axis([0 2*pi -Inf Inf])
    
    
    GradRadRmse(j,1) = sqrt(mean((powEstGrad-powGrad).^2));
    GradRadRmse(j,2) = sqrt(mean((powEstNoParaGrad-powGrad).^2));
    GradRadRmse(j,3) = sqrt(mean((powEstNoProfGrad-powGrad).^2));
    GradRadRmse(j,4) = sqrt(mean((powEstNoIndGrad-powGrad).^2));
    GradRadRmse(j,5) = sqrt(mean((powEstNoClimbGrad-powGrad).^2));
    
end

%% Plot single experiment relative error and gradient
exp = 6;

orbitsTemp = [];
index = 1;
while orbitPositions{1,exp}(index,1)<orbitPositions{1,exp}(index+1,1)
    index = index + 1;
end
index = index+1;
nextStartIdx = index;
for i=index:length(orbitPositions{1,exp})-1
    if orbitPositions{1,exp}(i,1)>orbitPositions{1,exp}(i+1,1)
        orbitsTemp = [orbitsTemp;1 exp nextStartIdx i];
        nextStartIdx = i+1;
    end
end
orbitsToAnalyze = orbitsTemp;

% orbitsToAnalyze = [ 1 6 36 105;
%                    1 6 106 173];

% window size
s = 0.5;
% radian resolution
r = 20;
rad = linspace(0, 2*pi, r);
spacing = 2*pi/(r-1);
xAxis = zeros(size(rad));

powAvg = zeros(r, size(orbitsToAnalyze, 1));
powEstAvg = zeros(r, size(orbitsToAnalyze, 1));
powEstNoParaAvg = zeros(r, size(orbitsToAnalyze, 1));
powEstNoProfAvg = zeros(r, size(orbitsToAnalyze, 1));
powEstNoIndAvg = zeros(r, size(orbitsToAnalyze, 1));
powEstNoClimbAvg = zeros(r, size(orbitsToAnalyze, 1));
for i =1:size(orbitsToAnalyze, 1)
    a = orbitsToAnalyze(i, 1);
    b = orbitsToAnalyze(i, 2);
    c = orbitsToAnalyze(i, 3);
    d = orbitsToAnalyze(i, 4);
    
    powAvg(:, i) = getAvgPowOfRad(flip(orbitPositions{a,b}(c:d)),...
        flip(powAll{a,b}(c:d)), s, r);
    
    powEstAvg(:, i) = getAvgPowOfRad(flip(orbitPositions{a,b}(c:d)),...
        flip(allPowerData{a,b}(c:d,1)), s, r);
    
    powEstNoParaAvg(:, i) = getAvgPowOfRad(flip(orbitPositions{a,b}(c:d)),...
        flip(allPowerData{a,b}(c:d,2)), s, r);
    
    powEstNoProfAvg(:, i) = getAvgPowOfRad(flip(orbitPositions{a,b}(c:d)),...
        flip(allPowerData{a,b}(c:d,3)), s, r);
    
    powEstNoIndAvg(:, i) = getAvgPowOfRad(flip(orbitPositions{a,b}(c:d)),...
        flip(allPowerData{a,b}(c:d,4)), s, r);
    
    powEstNoClimbAvg(:, i) = getAvgPowOfRad(flip(orbitPositions{a,b}(c:d)),...
        flip(allPowerData{a,b}(c:d,5)), s, r);
end

powRollMean = mean(powAvg, 2);
powEstRollMean = mean(powEstAvg, 2);
powEstNoParaRollMean = mean(powEstNoParaAvg, 2);
powEstNoProfRollMean = mean(powEstNoProfAvg, 2);
powEstNoIndRollMean = mean(powEstNoIndAvg, 2);
powEstNoClimbRollMean = mean(powEstNoClimbAvg, 2);

figure(16)
%{
for i =1:size(orbitsToAnalyze, 1)
    plot(rad, abs(powEstAvg(:,i) - powAvg(:,i))./powAvg(:,i)*100, '--b',...
        'LineWidth', 0.25);
    hold on
end
%}
p2 = plot(rad, abs(powEstRollMean - powRollMean)./powRollMean*100, '-b',...
    rad, abs(powEstNoParaRollMean - powRollMean)./powRollMean*100, '-g',...
    rad, abs(powEstNoProfRollMean - powRollMean)./powRollMean*100, '-r',...
    rad, abs(powEstNoIndRollMean - powRollMean)./powRollMean*100, '-m',...
    rad, abs(powEstNoClimbRollMean - powRollMean)./powRollMean*100, '-y',...
    'LineWidth', 2);
    xlabel('Position in orbit (rad)')
    ylabel('Instantaneous power relative error (%)')
    legend(p2, {
        'mean $P_{est}$ full model',...
        'mean $P_{est}$ no parasitic',...
        'mean $P_{est}$ no profile',...
        'mean $P_{est}$ no induced',...
        'mean $P_{est}$ no climb'},...
        'Location', 'northeast',...
        'Interpreter', 'latex')
    set(gca, 'fontsize', 18);
    axis([0 2*pi -Inf Inf])

figure(17)

% for i =1:size(orbitsToAnalyze, 1)
%     powGrad = gradient(powAvg(:,i),spacing);
%     %powEstGrad = gradient(powEstAvg(:,i),spacing);
%     %powEstNoParaGrad = gradient(powEstNoParaAvg(:,i),spacing);
%     %powEstNoProfGrad = gradient(powEstNoProfAvg(:,i),spacing);
%     %powEstNoIndGrad = gradient(powEstNoIndAvg(:,i),spacing);
%     %powEstNoClimbGrad = gradient(powEstNoClimbAvg(:,i),spacing);
%     plot(rad, powGrad, '--k',...
%         'LineWidth', 0.25);
%     hold on
% end

powGrad = gradient(powRollMean,spacing);
powEstGrad = gradient(powEstRollMean,spacing);
powEstNoParaGrad = gradient(powEstNoParaRollMean,spacing);
powEstNoProfGrad = gradient(powEstNoProfRollMean,spacing);
powEstNoIndGrad = gradient(powEstNoIndRollMean,spacing);
powEstNoClimbGrad = gradient(powEstNoClimbRollMean,spacing);
p1 = plot(rad, powGrad, '-k',...
        rad, powEstGrad, '-b',...
        rad, powEstNoParaGrad, '-g',...
        rad, powEstNoProfGrad, '-r',...
        rad, powEstNoIndGrad, '-m',...
        rad, powEstNoClimbGrad, '-y',...
        rad, xAxis,'--',...
        'LineWidth', 2);
    xlabel('Position in orbit (rad)')
    ylabel('Gradient (W/rad)')
    legend(p1, {
        'mean $P$ over all orbits', ...
        'mean $P_{est}$ full model',...
        'mean $P_{est}$ no parasitic',...
        'mean $P_{est}$ no profile',...
        'mean $P_{est}$ no induced',...
        'mean $P_{est}$ no climb'},...
        'Location', 'southeast',...
        'Interpreter', 'latex')    
    set(gca, 'fontsize', 18);
    axis([0 2*pi -Inf Inf])

%% Plot a whole flight
homePt = processedFltData{1, 8}{6, 1};
lat = processedFltData{1, 8}{7, 1};
lon = processedFltData{1, 8}{8, 1};
rx = zeros(size(lat));
ry = zeros(size(lon));
r = zeros(size(rx));
orbitPos = zeros(size(rx));

for i=1:length(rx)
    [rx(i), ry(i)] = GPStoRadius(homePt(1), homePt(2), lat(i), lon(i));
    r(i) = sqrt(rx(i)^2 + ry(i)^2);
    if ry(i) < 0
        orbitPos(i) = 2*pi - acos(rx(i)/r(i));
    else
        orbitPos(i) = acos(rx(i)/r(i));
    end
end

subplot(1,2,1)
plot(lon, lat)
title('In GPS coodinates')

subplot(1,2,2)
plot(rx, ry)
title('In x and y world coodinates')

%% Plot wind magnitude and heading as a function of time for one experiment
exp = 24;
startIdx = processedFltData{1,exp}{4,1};
endIdx = processedFltData{1,exp}{5,1};

airSpeedx = processedFltData{1, exp}{13, 1};
airSpeedy = processedFltData{1, exp}{14, 1};
velN = processedFltData{1, exp}{20, 1};
velE = processedFltData{1, exp}{21, 1};

% planar wind vector and norm
v_Win0 = [(airSpeedx)'; (airSpeedy)'];
v_Wnorm = zeros(size(v_Win0,2), 1);
for i=1:length(v_Wnorm)
    v_Wnorm(i) = sqrt(v_Win0(1,i)^2 + v_Win0(2,i)^2);
end

% wind heading
windHead = atan2(v_Win0(2,:), v_Win0(1,:)); 

x0 = processedFltData{1, exp}{23, 1}(startIdx);
xf = processedFltData{1, exp}{23, 1}(endIdx);
subplot(2,1,1)
plot(processedFltData{1, exp}{23, 1}(startIdx:endIdx),...
     v_Wnorm(startIdx:endIdx))
axis([x0 xf -Inf Inf])

subplot(2,1,2)
plot(processedFltData{1, exp}{23, 1}(startIdx:endIdx),...
     windHead(startIdx:endIdx))
axis([x0 xf -pi pi])


%% Plot absolute power curve vs target ground velocity
lightWindAbsPower = zeros(5,3);

for i=1:size(lightWindAbsPower,1)
    lightWindAbsPower(i,1) = processedFltData{1, 11+i}{14, 1}; % target linvel
    lightWindAbsPower(i,2) = processedFltData{1, 11+i}{11, 1}; % mean power
    lightWindAbsPower(i,3) = processedFltData{1, 11+i}{12, 1}; % power std
end

errorbar(lightWindAbsPower(:,1), lightWindAbsPower(:,2),...
    lightWindAbsPower(:,3), '-s','MarkerSize',10,'MarkerEdgeColor',...
    'red','MarkerFaceColor','red', 'LineStyle', 'none')
axis([-1 10 340 410])

%% plot moving mean of power vs time
% find the moving mean and std with a 5 second rolling window
startL = processedFltData{1, 24}{4, 1};
endL = processedFltData{1, 24}{5, 1};
startH = processedFltData{1, 25}{4, 1};
endH = processedFltData{1, 25}{5, 1};

powL = processedFltData{1, 24}{10, 1}(startL:200:endL);
powH = processedFltData{1, 25}{10, 1}(startH:200:endH);

powMovMeanL = movmean(powL, 5);
powMovMeanH = movmean(powH, 5);

if length(powMovMeanL) > length(powMovMeanH)
    t = 0:1:length(powMovMeanH) - 1;
    powMovMeanL = powMovMeanL(1:length(powMovMeanH));
elseif length(powMovMeanL) < length(powMovMeanH)
    t = 0:1:length(powMoveMeanL) - 1;
    powMovMeanH = powMovMeanH(1:length(powMovMeanL));
else
    t = 0:1:length(powMoveMeanH) - 1;
end

plot(t, powMovMeanL, '-r', t, powMovMeanH, '-b')

 %% plot moving mean of power vs position in orbit
startL = processedFltData{1, 13}{4, 1};
endL = processedFltData{1, 13}{5, 1};
startH = processedFltData{1, 8}{4, 1};
endH = processedFltData{1, 8}{5, 1};

powL = processedFltData{1, 13}{10, 1}(startL:200:endL);
powH = processedFltData{1, 8}{10, 1}(startH:200:endH);

homePtL = processedFltData{1, 13}{6, 1};
latL = processedFltData{1, 13}{7, 1}(startL:200:endL);
lonL = processedFltData{1, 13}{8, 1}(startL:200:endL);
rxL = zeros(size(latL));
ryL = zeros(size(lonL));
rL = zeros(size(rxL));
orbitPosL = zeros(size(rxL));

for i=1:length(rxL)
    [rxL(i), ryL(i)] = GPStoRadius(homePtL(1), homePtL(2), latL(i), lonL(i));
    rL(i) = sqrt(rxL(i)^2 + ryL(i)^2);
    if ryL(i) < 0
        orbitPosL(i) = 2*pi - acos(rxL(i)/rL(i));
    else
        orbitPosL(i) = acos(rxL(i)/rL(i));
    end
end

homePt = processedFltData{1, 8}{6, 1};
lat = processedFltData{1, 8}{7, 1}(startH:200:endH);
lon = processedFltData{1, 8}{8, 1}(startH:200:endH);
rx = zeros(size(lat));
ry = zeros(size(lon));
r = zeros(size(rx));
orbitPos = zeros(size(rx));

for i=1:length(rx)
    [rx(i), ry(i)] = GPStoRadius(homePt(1), homePt(2), lat(i), lon(i));
    r(i) = sqrt(rx(i)^2 + ry(i)^2);
    if ry(i) < 0
        orbitPos(i) = 2*pi - acos(rx(i)/r(i));
    else
        orbitPos(i) = acos(rx(i)/r(i));
    end
end

% find mean of each orbit
ltOrbitAvg = mean([powL(10:26) powL(27:43) powL(44:60)], 2);
hvyOrbitAvg = mean([powH(10:26) powH(27:43)], 2);

plot(orbitPosL(10:26), powL(10:26), '--r',...
     orbitPosL(27:43), powL(27:43), '--r',...
     orbitPosL(44:60), powL(44:60), '--r',...
     orbitPosL(44:60), ltOrbitAvg, '-r',...
     orbitPos(10:26), powH(10:26), '--b',...
     orbitPos(27:43), powH(27:43), '--b',...
     orbitPos(27:43), hvyOrbitAvg, '-b')
xlabel('Position in orbit (rad)')
ylabel('Instantaneous power (W)')
legend('wind 0-3 m/s', 'wind 8-11 m/s', 'Location', 'northwest')

%% Determine drag coefficients from all data
load('processedFltDataBackupPreAblation.mat')
N = 17;

vdot_in0 = cell(1, N);
thrustEst = cell(1, N);
dragEst = cell(1, N);
Wvec = params.m*params.g*[0; 0; -1];

for i=1:N
    vdot_in0{1, i} = getvdot(processedFltDataBackupPreAblation{1, i});
    thrustEst{1, i} = getThrustEst(processedFltDataBackupPreAblation{1, i}, params);
end

for j=1:N
    startIdx = processedFltDataBackupPreAblation{1,j}{4,1};
    endIdx = processedFltDataBackupPreAblation{1,j}{5,1};
    
    v_inf = processedFltDataBackupPreAblation{1, j}{24,1}(startIdx:endIdx,:);
    That = thrustEst{1, j}(startIdx:endIdx,:);
    vdot = vdot_in0{1, j}(startIdx:endIdx,:);
    draghat = zeros(size(v_inf, 1), 5);
    
    for k=1:length(draghat) - 1
        dragvec = (params.m*vdot(k,:)' - That(k,:)' - Wvec)';
        draghat(k,:) = [v_inf(k,:), dragvec, norm(dragvec)];
    end
    
    dragEst{1, j} = draghat;
end

sctrx = [dragEst{1,1}(1:40:end,1);
         dragEst{1,2}(1:40:end,1);
         dragEst{1,3}(1:40:end,1);
         dragEst{1,4}(1:40:end,1);
         dragEst{1,6}(1:40:end,1);
         dragEst{1,7}(1:40:end,1);
         dragEst{1,8}(1:40:end,1);
         dragEst{1,9}(1:40:end,1);
         dragEst{1,13}(1:40:end,1);
         dragEst{1,14}(1:40:end,1);
         dragEst{1,15}(1:40:end,1);
         dragEst{1,16}(1:40:end,1);
         dragEst{1,17}(1:40:end,1);];
sctry = [dragEst{1,1}(1:40:end,5);
         dragEst{1,2}(1:40:end,5);
         dragEst{1,3}(1:40:end,5);
         dragEst{1,4}(1:40:end,5);
         dragEst{1,6}(1:40:end,5);
         dragEst{1,7}(1:40:end,5);
         dragEst{1,8}(1:40:end,5);
         dragEst{1,9}(1:40:end,5);
         dragEst{1,13}(1:40:end,5);
         dragEst{1,14}(1:40:end,5);
         dragEst{1,15}(1:40:end,5);
         dragEst{1,16}(1:40:end,5);
         dragEst{1,17}(1:40:end,5);];

scatter(sctrx, sctry)
xlabel('Relative velocity norm $v_\infty$ (m/s)', 'Interpreter', 'latex')
ylabel('Estimated drag force norm $\hat{f}_D$ (N)', 'Interpreter', 'latex')
set(gca, 'fontsize', 18);

%% Data file index reference guide
% GPS data at 200 Hz
% IMU data at 200 Hz
% Mag data at 50 Hz
% Battery data at 1 Hz
% 1 - flt#_dayMonth
% 2 - total number of data points
% 3 - velocity norm for entire flight
% 4 - experiment starting index
% 5 - experiment ending index
% 6 - gps home lat and long
% 7 - gps lat for experiment
% 8 - gps long for experiment
% 9 - gps relative height from homepoint
% 10 - instaneous power for experiment
% 11 - mean power for flight
% 12 - standard deviation of power for flight
% 13 - windspeed x in ground frame
% 14 - windspeed y in ground frame
% 15 - windspeed x in body frame
% 16 - windspeed y in body frame
% 17 - wind condition
% 18 - target linear velocity
% 19 - mission type
% 20 - gps velocity north
% 21 - gps velocity east
% 22 - gps velocity down
% 23 - offset time
% 24 - air speed vel norm in body?
% 25 - roll (deg)
% 26 - pitch (deg)
% 27 - yaw (deg)
% 28 - imu vel N
% 29 - imu vel E
% 30 - imu vel D