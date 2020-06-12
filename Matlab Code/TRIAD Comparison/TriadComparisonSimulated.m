clear all
close all
clc
%% THIS FILE COMPARES THE DCM USING CLOSED LOOP ON THE BOARD, AND THE DCM USING THE TRIAD MATLAB FUNCTION HERE
%% Constants
accelInertial = [0;0;1];
He = [22770;5329;41510.2]/1000;     % Earth's magnetic field in uT (NED)
magInertial = He/norm(He);

%% Import Sensor Data and Experimental DCM
DATA = csvread('data/accel_mag_dcm_TUMBLE.csv');
% Read sensor dat
j=1;
k=2; %index of DATA file
l=1; %Index of array Rexp
for i=1:length(DATA)/4
    sensor_data(i,:)=DATA(j,:);
    
    
    Rexp(l,:)=DATA(k,:);
    Rexp(l+1,:)=DATA(k+1,:);
    Rexp(l+2,:)=DATA(k+2,:);
    l=l+3;
    j=j+4;
    k=k+4;
end
Rexp = Rexp(:,1:3);
mags = sensor_data(:,1:3);
accels = sensor_data(:,4:6);

%% Use Triad to create R_triad
j=1;
for i=1:length(mags)
    
    R_triad(j:j+2,:) = DCMfromTriad(mags(i,:)', accels(i,:)', magInertial, accelInertial);
    j=j+3;

end

%% Get Euler angles
j=1;
for i=1:length(R_triad)/3
    R = R_triad(j:j+2,:);
    theta_triad(i)=asin(-R(1,3));
    phi_triad(i)=atan2(R(2,3),R(3,3)); % atan2() accounts for which quadrant angle should be
    psi_triad(i)=atan2(R(1,2),R(1,1));

    R = Rexp(j:j+2,:);
    theta_exp(i)=asin(-R(1,3));
    phi_exp(i)=atan2(R(2,3),R(3,3)); % atan2() accounts for which quadrant angle should be
    psi_exp(i)=atan2(R(1,2),R(1,1));
    
    j=j+3;
    
end

%% Organize Euler angels
Eul_exp = [psi_exp; theta_exp; phi_exp];
Eul_triad = [psi_triad; theta_triad; phi_triad];
Eul_exp = rad2deg(Eul_exp');
Eul_triad = rad2deg(Eul_triad');

%% CORRECTIONS
Eul_exp(:,1) = Eul_exp(:,1)*-1 ; %Yaw is inverted for some reason
Eul_exp(:,2) = Eul_exp(:,2)*-1 ; %Yaw is inverted for some reason
Eul_exp(:,3) = Eul_exp(:,3)*-1 ; %Yaw is inverted for some reason

%% Experimental v. TRIAD Euler Angles
t = 1:length(Eul_exp);
t = t/50;
figure()
subplot(311)
plot(t,Eul_exp(:,1));
hold on
plot(t,Eul_triad(:,1));
legend('Yaw Experimental \psi', 'Yaw TRIAD \psi')
xlabel('Time');
ylabel('Degrees')
title('CLOSED LOOP EXPERIMENTAL V. TRIAD EULER ANGLES')
hold on
subplot(312)
plot(t,Eul_exp(:,2));
hold on
plot(t,Eul_triad(:,2));
legend('Pitch Experimental \theta', 'Pitch TRIAD \theta')
xlabel('Time');
ylabel('Degrees')
hold on 
subplot(313)
plot(t,Eul_exp(:,3));
hold on
plot(t,Eul_triad(:,3));
legend('Roll Experimental \phi', 'Roll TRIAD \phi')
xlabel('Time');
ylabel('Degrees')

%% Error Comparison
figure();
plot(t,Eul_exp - Eul_triad)
xlabel('Time');
ylabel('Degrees')
legend('Yaw \psi','Pitch \theta','Roll \phi')
title('Error Experimental v. TRIAD')

error = [Eul_exp-Eul_triad] ;
std = std(error);
mean = mean(real(abs(error)));

figure()
subplot(311)
title('Error Histogram TRIAD v. Closed Loop AHRS Filter')
hold on
histfit(error(:,1))
legend('Yaw(\psi) Error ')
subplot(312)
histfit(error(:,2))
legend('Pitch(\theta) Error')
subplot(313)
histfit(error(:,3))
legend('Roll(\phi) Error')
