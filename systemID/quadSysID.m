% index guide
% 2 - offset time (s)
% 14 - gyro z (deg/s)
% 22 - imu d velocity (m/s)
% 26 - roll (deg)
% 27 - pitch (deg)
% 28 - yaw (deg)
% 57 - gps d velocity (m/s)
% 58 - scaled roll input [-10000, 10000]
% 59 - scaled pitch input [-10000, 10000]
% 60 - scaled yaw input [-10000, 10000]
% 61 - scaled thrust input [-10000, 10000]
clear

load('sysID_30April.mat')

% time vector
t = sysID_30April(:,2);
Ts = mean(diff(sysID_30April(:,2)));

% Attitude scaled input conversion to radians Here we assume that max input
% is equivalent to max roll/pitch angle. From DJI we know that is 35 deg or
% 0.61086524 rad for pitch and roll and 150 deg/s or 2.61799387 rad/s for
% yaw rate.
kRollPitch = 0.61086524/10000;
uRoll = kRollPitch*sysID_30April(:,58);
uPitch = kRollPitch*sysID_30April(:,59);

kYawd = 2.61799387/10000;
uYawd = kYawd*sysID_30April(:,60);

% convert fused attitude measurements from the imu to radians
roll = deg2rad(sysID_30April(:,26));
pitch = deg2rad(sysID_30April(:,27));
yawd = deg2rad(sysID_30April(:,14));

% Assume that thrust command is proportional to actual z velocity and use
% max ascent, 5 m/s, and descent, 4 m/s, to scale.
uThrust = zeros(size(sysID_30April(:,61)));
for i=1:length(sysID_30April(:,61))
    if sysID_30April(i,61) < 0
        uThrust(i) = 4/10000*sysID_30April(i,61);
    else
        uThrust(i) = 5/10000*sysID_30April(i,61);
    end
end

velD = sysID_30April(:,22);
%% Identify experiments by index
% done by looking for isolated input values in csv file
% don't forget to subtract 1 from the row number in excel
% goal is to have 6 of each, 5 training sets and 1 validation set

% pitch 
pExp1.u = uPitch(1139:1230);
pExp1.y = pitch(1139:1230);

pExp2.u = uPitch(4155:4464);
pExp2.y = pitch(4155:4464);

pExp3.u = uPitch(6695:6927);
pExp3.y = pitch(6695:6927);

pExp4.u = uPitch(8581:8927);
pExp4.y = pitch(8581:8927);

pExp5.u = uPitch(9184:9327);
pExp5.y = pitch(9184:9327);

pExp6.u = uPitch(10553:10789);
pExp6.y = pitch(10553:10789);

% roll
rExp1.u = uRoll(1895:2007);
rExp1.y = roll(1895:2007);

rExp2.u = uRoll(2049:2363);
rExp2.y = roll(2049:2363);

rExp3.u = uRoll(5268:5577);
rExp3.y = roll(5268:5577);

rExp4.u = uRoll(6540:6661);
rExp4.y = roll(6540:6661);

rExp5.u = uRoll(8080:8189);
rExp5.y = roll(8080:8189);

rExp6.u = uRoll(28165:28430);
rExp6.y = roll(28165:28430);

% yaw rate
ydExp1.u = uYawd(12574:13515);
ydExp1.y = yawd(12574:13515);

ydExp2.u = uYawd(33940:34228);
ydExp2.y = yawd(33940:34228);

ydExp3.u = uYawd(34425:34805);
ydExp3.y = yawd(34425:34805);

ydExp4.u = uYawd(34928:35327);
ydExp4.y = yawd(34928:35327);

ydExp5.u = uYawd(36394:36618);
ydExp5.y = yawd(36394:36618);

ydExp6.u = uYawd(37477:37727);
ydExp6.y = yawd(37477:37727);

% z velocity
vExp1.u = uThrust(41198:41371);
vExp1.y = velD(41198:41371);

vExp2.u = uThrust(1465:1556);
vExp2.y = velD(1465:1556);

vExp3.u = uThrust(39082:39248);
vExp3.y = velD(39082:39248);

vExp4.u = uThrust(6004:6179);
vExp4.y = velD(6004:6179);

vExp5.u = uThrust(11192:11317);
vExp5.y = velD(11192:11317);

vExp6.u = uThrust(39686:39898);
vExp6.y = velD(39686:39898);

%% Create and merge data objects
roll1 = iddata(rExp1.y,rExp1.u,Ts,...
    'ExperimentName', 'roll1', 'InputName','roll_{cmd}',...
    'OutputName','roll', 'InputUnit','rad', 'OutputUnit','rad',...
    'TimeUnit','Second');
roll2 = iddata(rExp2.y,rExp2.u,Ts,...
    'ExperimentName', 'roll2', 'InputName','roll_{cmd}',...
    'OutputName','roll', 'InputUnit','rad', 'OutputUnit','rad',...
    'TimeUnit','Second');
roll3 = iddata(rExp3.y,rExp3.u,Ts,...
    'ExperimentName', 'roll3', 'InputName','roll_{cmd}',...
    'OutputName','roll', 'InputUnit','rad', 'OutputUnit','rad',...
    'TimeUnit','Second');
roll4 = iddata(rExp4.y,rExp4.u,Ts,...
    'ExperimentName', 'roll4', 'InputName','roll_{cmd}',...
    'OutputName','roll', 'InputUnit','rad', 'OutputUnit','rad',...
    'TimeUnit','Second');
roll5 = iddata(rExp5.y,rExp5.u,Ts,...
    'ExperimentName', 'roll5', 'InputName','roll_{cmd}',...
    'OutputName','roll', 'InputUnit','rad', 'OutputUnit','rad',...
    'TimeUnit','Second');
roll6 = iddata(rExp6.y,rExp6.u,Ts,...
    'ExperimentName', 'roll6', 'InputName','roll_{cmd}',...
    'OutputName','roll', 'InputUnit','rad', 'OutputUnit','rad',...
    'TimeUnit','Second');
rollData = detrend(merge(roll1, roll2, roll3, roll4, roll5, roll6));

pitch1 = iddata(pExp1.y,pExp1.u,Ts,...
    'ExperimentName', 'pitch1', 'InputName','pitch_{cmd}',...
    'OutputName','pitch', 'InputUnit','rad', 'OutputUnit','rad',...
    'TimeUnit','Second');
pitch2 = iddata(pExp2.y,pExp2.u,Ts,...
    'ExperimentName', 'pitch2', 'InputName','pitch_{cmd}',...
    'OutputName','pitch', 'InputUnit','rad', 'OutputUnit','rad',...
    'TimeUnit','Second');
pitch3 = iddata(pExp3.y,pExp3.u,Ts,...
    'ExperimentName', 'pitch3', 'InputName','pitch_{cmd}',...
    'OutputName','pitch', 'InputUnit','rad', 'OutputUnit','rad',...
    'TimeUnit','Second');
pitch4 = iddata(pExp4.y,pExp4.u,Ts,...
    'ExperimentName', 'pitch4', 'InputName','pitch_{cmd}',...
    'OutputName','pitch', 'InputUnit','rad', 'OutputUnit','rad',...
    'TimeUnit','Second');
pitch5 = iddata(pExp5.y,pExp5.u,Ts,...
    'ExperimentName', 'pitch5', 'InputName','pitch_{cmd}',...
    'OutputName','pitch', 'InputUnit','rad', 'OutputUnit','rad',...
    'TimeUnit','Second');
pitch6 = iddata(pExp6.y,pExp6.u,Ts,...
    'ExperimentName', 'pitch6', 'InputName','pitch_{cmd}',...
    'OutputName','pitch', 'InputUnit','rad', 'OutputUnit','rad',...
    'TimeUnit','Second');
pitchData = detrend(merge(pitch1, pitch2, pitch3, pitch4, pitch5, pitch6));

yawd1 = iddata(ydExp1.y,ydExp1.u,Ts,...
    'ExperimentName', 'yawd1', 'InputName','yawd_{cmd}',...
    'OutputName','yawd', 'InputUnit','rad', 'OutputUnit','rad',...
    'TimeUnit','Second');
yawd2 = iddata(ydExp2.y,ydExp2.u,Ts,...
    'ExperimentName', 'yawd2', 'InputName','yawd_{cmd}',...
    'OutputName','yawd', 'InputUnit','rad', 'OutputUnit','rad',...
    'TimeUnit','Second');
yawd3 = iddata(ydExp3.y,ydExp3.u,Ts,...
    'ExperimentName', 'yawd3', 'InputName','yawd_{cmd}',...
    'OutputName','yawd', 'InputUnit','rad', 'OutputUnit','rad',...
    'TimeUnit','Second');
yawd4 = iddata(ydExp4.y,ydExp4.u,Ts,...
    'ExperimentName', 'yawd4', 'InputName','yawd_{cmd}',...
    'OutputName','yawd', 'InputUnit','rad', 'OutputUnit','rad',...
    'TimeUnit','Second');
yawd5 = iddata(ydExp5.y,ydExp5.u,Ts,...
    'ExperimentName', 'yawd5', 'InputName','yawd_{cmd}',...
    'OutputName','yawd', 'InputUnit','rad', 'OutputUnit','rad',...
    'TimeUnit','Second');
yawd6 = iddata(ydExp6.y,ydExp6.u,Ts,...
    'ExperimentName', 'yawd6', 'InputName','yawd_{cmd}',...
    'OutputName','yawd', 'InputUnit','rad', 'OutputUnit','rad',...
    'TimeUnit','Second');
yawdData = detrend(merge(yawd1, yawd2, yawd3, yawd4, yawd5, yawd6));

velD1 = iddata(vExp1.y,vExp1.u,Ts,...
    'ExperimentName', 'velD1', 'InputName','velD_{cmd}',...
    'OutputName','velD', 'InputUnit','rad', 'OutputUnit','rad',...
    'TimeUnit','Second');
velD2 = iddata(vExp2.y,vExp2.u,Ts,...
    'ExperimentName', 'velD2', 'InputName','velD_{cmd}',...
    'OutputName','velD', 'InputUnit','rad', 'OutputUnit','rad',...
    'TimeUnit','Second');
velD3 = iddata(vExp3.y,vExp3.u,Ts,...
    'ExperimentName', 'velD3', 'InputName','velD_{cmd}',...
    'OutputName','velD', 'InputUnit','rad', 'OutputUnit','rad',...
    'TimeUnit','Second');
velD4 = iddata(vExp4.y,vExp4.u,Ts,...
    'ExperimentName', 'velD4', 'InputName','velD_{cmd}',...
    'OutputName','velD', 'InputUnit','rad', 'OutputUnit','rad',...
    'TimeUnit','Second');
velD5 = iddata(vExp5.y,vExp5.u,Ts,...
    'ExperimentName', 'velD5', 'InputName','velD_{cmd}',...
    'OutputName','velD', 'InputUnit','rad', 'OutputUnit','rad',...
    'TimeUnit','Second');
velD6 = iddata(vExp6.y,vExp6.u,Ts,...
    'ExperimentName', 'velD6', 'InputName','velD_{cmd}',...
    'OutputName','velD', 'InputUnit','rad', 'OutputUnit','rad',...
    'TimeUnit','Second');
velDData = detrend(merge(velD1, velD2, velD3, velD4, velD5, velD6));

%% Estimate pitch dynamics
% training data
pTrain = getexp(pitchData, [2,3,4,5,6]);

% validation set
pVal = getexp(pitchData, 1);

% compare different estimation methods
model1 = arx(pTrain,[2 2 1]);
model2 = n4sid(pTrain,2);
model3 = armax(pTrain,[2 2 2 1]);
model4 = tfest(pTrain,2,1);

compare(pVal,model1,model2,model3,model4)

pitchModel = model3;
%% Estimate roll dynamics
% training data
rTrain = getexp(rollData, [1,3,4,5,6]);

% validation set
rVal = getexp(rollData, 2);

% compare different estimation methods
model1 = arx(rTrain,[2 2 1]);
model2 = n4sid(rTrain,2);
model3 = armax(rTrain,[2 2 2 1]);
model4 = tfest(rTrain,2,1);

compare(rVal,model1,model2,model3,model4)

rollModel = model3;
%% Estimate yaw rate dynamics
% training data
ydTrain = getexp(yawdData, [1,2,4,5,6]);

% validation set
ydVal = getexp(yawdData, 3);

% compare different estimation methods
model1 = arx(ydTrain,[2 2 1]);
model2 = n4sid(ydTrain,2);
model3 = armax(ydTrain,[2 2 2 1]);
model4 = tfest(ydTrain,2,1);

compare(ydVal,model1,model2,model3,model4)

yawdModel = model1;
%% Estimate z velocity dynamics
% training data
vTrain = getexp(velDData, [1,2,3,5,6]);

% validation set
vVal = getexp(velDData, 4);

% compare different estimation methods
model1 = arx(vTrain,[2 2 1]);
model2 = n4sid(vTrain,2);
model3 = armax(vTrain,[2 2 2 1]);
model4 = tfest(vTrain,2,1);

compare(vVal,model1,model2,model3,model4)

velDModel = model3;