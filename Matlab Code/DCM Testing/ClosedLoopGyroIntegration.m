clear all;
close all;
clc;

numSteps=400;

accelInertial=[0;0;-1];
accelReading=[0;0;-1];
He = [22770;5329;41510.2]/1000;     % Earth's magnetic field in uT (NED)
magInertial = He/norm(He);
magReading=[1;0;0];

dt=.02;
AccelScale = 16834;

biasEstimate = [0;0;0]

[gyrobias, Acc,Mag,wGyro,Eul] = CreateTrajectoryData(dt,0)
% gyroRaw = wGyro;

% dat = csvread('closed Loop Accel/accelMagdata.csv');
% wGyro = [dat(:,1) dat(:,2) dat(:,3)]; 
% Mag = ones(length(wGyro),3);
% Acc = [dat(:,4)-108 dat(:,5)+185 dat(:,6)-917];
% Acc = Acc/AccelScale;

wGyro = deg2rad(wGyro/131)


% angleX = deg2rad(Eul(1,3));
% angleY = deg2rad(Eul(1,2));
% angleZ = deg2rad(Eul(1,1));

angleX = deg2rad(0);
angleY = deg2rad(0);
angleZ = deg2rad(0);

Ro=[cos(angleY)*cos(angleZ) cos(angleY)*sin(angleZ) -sin(angleY); 
sin(angleX)*sin(angleY)*cos(angleZ)-cos(angleX)*sin(angleZ) sin(angleX)*sin(angleY)*sin(angleZ)-cos(angleX)*cos(angleZ) sin(angleX)*cos(angleY);
    cos(angleX)*sin(angleY)*cos(angleZ)+sin(angleX)*sin(angleZ) cos(angleX)*sin(angleY)*sin(angleZ)-sin(angleX)*cos(angleZ) cos(angleX)*cos(angleY)]
Ro=eye(3);

R=Ro;



for i=1:50*8             

    [R, biasEstimate] = IntegrateClosedLoop(R, biasEstimate, wGyro(i,:)', Mag(i,:)', Acc(i,:)', magInertial, accelInertial, dt);
    
    theta(i)=asin(-R(1,3));
    phi(i)=atan2(R(2,3),R(3,3)); % atan2() accounts for which quadrant angle should be
    psy(i)=atan2(R(1,2),R(1,1));




end

%% Integrated Euler Angles
theta = rad2deg(theta);
psy = rad2deg(psy);
phi = rad2deg(phi);

figure()
t = 1:length(theta);
t=t/50;
% plot(t,psy)
% hold on
plot(t, theta)
hold on 
plot(t,phi)
xlabel('Time');
ylabel('Degrees')
legend('Pitch \theta','Roll \phi')
title('Closed Loop Using Accelerometer Data')

%% Real Euler Angless
figure()
plot(t,Eul(1:length(theta),1));
hold on
plot(t,Eul(1:length(theta),2));
hold on 
plot(t,Eul(1:length(theta),3));
xlabel('Time');
ylabel('Degrees')
legend('Yaw \psi','Pitch \theta','Roll \phi')
title('Real Euler Angles')

%% Error Comparison
error = ones(length(theta),3);
error(:,1)=psy'-Eul(1:length(theta),1);
error(:,2)=theta'-Eul(1:length(theta),2);
error(:,3)=phi'-Eul(1:length(theta),3);

figure();
plot(t,error(:,1));
hold on
plot(t,error(:,2));
hold on
plot(t,error(:,3));
xlabel('Time');
ylabel('Degrees')
legend('Yaw \psi','Pitch \theta','Roll \phi')
title('Error Real v. Calculated')

error = [error(:,1); error(:,2); error(:,3)] ;
std = std(error);
mean = mean(real(abs(error)));