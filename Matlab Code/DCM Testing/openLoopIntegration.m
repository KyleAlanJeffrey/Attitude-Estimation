clear all
close all
clc

%%

R = eye(3);

nvector=[1;0;0];
evector=[0;-1;0];
dvector=[0;0;-1];

[sX,sY,sZ]=sphere(30);
surf(sX,sY,sZ,'FaceAlpha',.1,'EdgeColor','none');
axis equal;
xlabel('x')
ylabel('y')
zlabel('z')
hold on
quiver3([0,0,0],[0,0,0],[0,0,0],[nvector(1),evector(1),dvector(1)],[nvector(2),evector(2),dvector(2)],[nvector(3),evector(3),dvector(3)]);
pause

%% Load CSV Data
Rexp_John = 'Matrix Exponential Data/JohnData.csv';
Rexp_Simulated = 'Matrix Exponential Data/Z_10_Rotation_Simulated.csv';
Rexp_Simulated_Longer = 'Matrix Exponential Data/Z_Rotation_Simulated_1.csv';
Rexp_Experimental = 'Matrix Exponential Data/Z_5_Rotation_Experimental.csv';
Rexp_Experimental_tumble = 'Matrix Exponential Data/tumbling_Experimental.csv';
closeLoop_Experimental = 'Close Loop Integration Data/closedLoop_Experimental.csv';
R_data = csvread(closeLoop_Experimental);
R_data = R_data(:,1:3)

%% Set Number of data points

numSteps = length(R_data) /3;

%% Graph DCM data point by datta point
j=1;
for i=1: numSteps
    R(1,:)=R_data(j,:);
    j=j+1;
    R(2,:)=R_data(j,:);
    j=j+1;
    R(3,:)=R_data(j,:);
    j=j+1;
%     disp(R)
    
    phi(i)=asin(-R(1,3));
    psy(i)=atan2(R(2,3),R(3,3)); % atan2() accounts for which quadrant angle should be
    theta(i)=atan2(R(1,2),R(1,1));

    newNVector=R'*nvector;
    newEVector=R'*evector;
    newDVector=R'*dvector;

    norms(i)= (norm(newNVector)+norm(newEVector)+norm(newDVector))/3;
%     quiver3([0,0,0],[0,0,0],[0,0,0],[newNVector(1),newEVector(1),newDVector(1)],[newNVector(2),newEVector(2),newDVector(2)],[newNVector(3),newEVector(3),newDVector(3)]);
end

%% Last vector
%     quiver3([0,0,0],[0,0,0],[0,0,0],[newNVector(1),newEVector(1),newDVector(1)],[newNVector(2),newEVector(2),newDVector(2)],[newNVector(3),newEVector(3),newDVector(3)]);

%% Orthonormality check

orthoError = R'*R
    
title('Forward Integration 45 Degree Rotation');
dT=.02;
t = 1:length(theta);
t=t*dT; % time in seconds

figure()
scatter(t,norms);
title('Norm of Body Frame Vectors');
xlabel('Time(s)');
ylabel('|R|');

theta = rad2deg(theta);
psy = rad2deg(psy);
phi = rad2deg(phi);

Eul = [psy' theta' phi'];
AnimateAttitude(dT, Eul)

figure()
plot(t,theta)
hold on
plot(t, psy)
hold on 
plot(t,phi)
xlabel('Time(s)');
ylabel('Degrees');
legend('\theta','\psi','\phi')