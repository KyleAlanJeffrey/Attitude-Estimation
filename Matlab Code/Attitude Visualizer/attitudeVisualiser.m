clear all;
close all;
clc;


dT=.02; %50hz

%% Get Experimental Data
wGyro = csvread('data/attitudeTest.csv');

%% Get angles step by step
j = 1;
for i=1:length(wGyro)/3
    R(1:3,:)=wGyro(j:j+2,:);
    j=j+3;
    theta(i)=asin(-R(1,3));
    phi(i)=atan2(R(2,3),R(3,3)); % atan2() accounts for which quadrant angle should be
    psi(i)=atan2(R(1,2),R(1,1));
    
Error = R'*R
    
end
Eul = [psi', theta', phi'];
Eul = rad2deg(Eul);
AnimateAttitude(dT, Eul);

%% Euler Angles over time
figure()
t=1:length(theta);
t=t/50;
% plot(t,Eul(1:length(theta),1))
% hold on
plot(t,Eul(1:length(theta),2))
hold on 
plot(t,Eul(1:length(theta),3))
xlabel('Time(s)');
ylabel('Degrees')
legend('Pitch \theta','Roll \phi')
title('Experimental Closed Loop w/ Accelerometer')

%% Orthonormality Check
