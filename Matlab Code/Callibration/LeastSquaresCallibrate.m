% clear all 
close all 
clc 

%% Import Data
dat = csvread('magAccelTumbleRaw.csv');
%% Scale factors
He = [22770;5329;41510.2]/1000;     % Earth's magnetic field in uT (NED)
Ge = [0;0;1];                       % Earth's gravitational field in g (NED)
AscaleFactor = 0.5*(2^15-1);        % conversion of g's to bits
HscaleFactor = 1/0.15;              % uT/bits

teslaScale = norm(He)


gScale = 1;
mx = dat(:,1);
my = dat(:,2);
mz = dat(:,3);
ax = dat(:,4);
ay = dat(:,5);
az = dat(:,6);

%% Plot originals 
figure(7)
scatter3(mx,my,mz);
xlabel('x');
ylabel('y');
zlabel('z');
title('Mag Raw Scaled')
axis equal
figure(8)
scatter3(ax,ay,az);
xlabel('x');
ylabel('y');
zlabel('z');
title('Accel Raw Scaled')
axis equal
%% Calibrate Data using Least Squares
kstep = 200;
plotFlag = 0;
figure()
[Atilde,Btilde] = CalibrateEllipsoidData3D(mx,my,mz,kstep,0);
[Xcorr,Ycorr,Zcorr] = CorrectEllipsoidData3D(mx,my,mz,Atilde,Btilde);
scatter3(Xcorr,Ycorr,Zcorr);
axis equal
disp(Atilde)
disp(Btilde)

%% Norm Check
% for i = 1: length(Hnoise)
%     p(i)=norm([Xcorr(i) Ycorr(i) Zcorr(i)]);
% end
% stdH = std(p)