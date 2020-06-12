function [error, S] = compareDCM(R1,R2)
%COMPAREDCM Compare the difference between angles of two DCM Rotation
%Matrices
%   Detailed explanation goes here
R_triad = R2;
Rexp = R1;
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
Eul_exp(:,2) = Eul_exp(:,2)*-1 ; %pitch is inverted for some reason
Eul_exp(:,3) = Eul_exp(:,3)*-1 ; %roll is inverted for some reason

%% Experimental v. TRIAD Euler Angles
t = 1:length(Eul_exp);
t = t/50;
figure()
subplot(311)
plot(t,Eul_exp(:,1));
hold on
plot(t,Eul_triad(:,1));
legend('Yaw Closed Loop \psi', 'Yaw TRIAD \psi')
xlabel('Time');
ylabel('Degrees')
title('CLOSED LOOP V. TRIAD EULER ANGLES')
hold on
subplot(312)
plot(t,Eul_exp(:,2));
hold on
plot(t,Eul_triad(:,2));
legend('Pitch Closed Loop \theta', 'Pitch TRIAD \theta')
xlabel('Time');
ylabel('Degrees')
hold on 
subplot(313)
plot(t,Eul_exp(:,3));
hold on
plot(t,Eul_triad(:,3));
legend('Roll Closed Loop \phi', 'Roll TRIAD \phi')
xlabel('Time');
ylabel('Degrees')

%% Error Comparison
figure();
plot(t,Eul_exp - Eul_triad)
xlabel('Time');
ylabel('Degrees')
legend('Yaw \psi','Pitch \theta','Roll \phi')
title('Error Closed Loop v. TRIAD')

error = [Eul_exp-Eul_triad] ;
S = std(error)
M = mean(real(abs(error)));

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

end

