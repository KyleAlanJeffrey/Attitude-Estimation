%% 
clear all
close all
clc

%% Import Tumble Data

dat = csvread('magAccelTumbleCalibrated.csv');
mag = dat(:,1:3);
acc = dat(:,4:6);

%% Constants
He = [22770;5329;41510.2]/1000;     % Earth's magnetic field in uT (NED)

magIntertial = He/norm(He);

accIntertial = [0;0;1];

%% do stuff
[Rmis, Pbody] = AlignMasterSlave(acc',mag',accIntertial,magIntertial,eye(3),200);
[lambda,phi] = extractAxis(Rmis)

theta=asin(-Rmis(1,3));
phi=atan2(Rmis(2,3),Rmis(3,3)); % atan2() accounts for which quadrant angle should be
psy=atan2(Rmis(1,2),Rmis(1,1));

%%
rad2deg(theta)
rad2deg(phi)
rad2deg(psy)
