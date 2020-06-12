clear all;
close all;
clc;

%% Grab Simulated Data
[Acc,Mag,wGyro,Eul] = CreateTrajectoryData(.02,'true');

%% Grabb bias if noise
xBias = mean(wGyro(1:50,3));
yBias = mean(wGyro(1:50,2));
zBias = mean(wGyro(1:50,1));

wGyro(:,1) = wGyro(:,1)-zBias;
wGyro(:,2) = wGyro(:,2)-yBias;
wGyro(:,3) = wGyro(:,3)-xBias;

wGyro=wGyro/131; %Scale factor

%% Use either constant Gyro or Created Trajectory Data for gyroinput
% p=deg2rad(0);
% q=deg2rad(0);
% r=deg2rad(1);
% gyroInput=[p;q;r]; %Constant values

gyroInput=[wGyro(:,1),wGyro(:,2),wGyro(:,3)]; %Trajectory Data
gyroInput=deg2rad(gyroInput);
dt=.02; %50hz

%% Determine starting DCM

angleX = deg2rad(Eul(1,3));
angleY = deg2rad(Eul(1,2));
angleZ = deg2rad(Eul(1,1));

Ro=[cos(angleY)*cos(angleZ) cos(angleY)*sin(angleZ) -sin(angleY); 
    sin(angleX)*sin(angleY)*cos(angleZ)-cos(angleX)*sin(angleZ) sin(angleX)*sin(angleY)*sin(angleZ)-cos(angleX)*cos(angleZ) sin(angleX)*cos(angleY);
    cos(angleX)*sin(angleY)*cos(angleZ)+sin(angleX)*sin(angleZ) cos(angleX)*sin(angleY)*sin(angleZ)-sin(angleX)*cos(angleZ) cos(angleX)*cos(angleY)]

% Ro=eye(3);
R=Ro; %Rotation matrix

%% Run through integration steps
for i=1:length(Eul)
    Rminus =R;
    [R] = IntegrateOpenLoop(Rminus, gyroInput(i,:), dt);
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
plot(t,psy)
hold on
plot(t, theta)
hold on 
plot(t,phi)
xlabel('Time');
ylabel('Degrees')
legend('Yaw \psi','Pitch \theta','Roll \phi')
title('Integrated Euler Angles')

%% Real Euler Angless
figure()
plot(t,Eul(1:length(theta),1))
hold on
plot(t,Eul(1:length(theta),2))
hold on 
plot(t,Eul(1:length(theta),3))
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

error = [error(:,1); error(:,2); error(:,3)] 
std = std(error)
mean = mean(real(abs(error)))