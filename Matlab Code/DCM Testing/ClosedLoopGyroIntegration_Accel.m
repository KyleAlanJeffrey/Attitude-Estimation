clear all;
close all;
clc;

numSteps=200;

p=deg2rad(0);
q=deg2rad(0);
r=deg2rad(0);
gyroInput=[p;q;r];

biasTerms=[.1;.1;.1];
% biasTerms=[.0;.0;.0];

accelInertial=[0;0;-1];
accelReading=[0;0;-1];


Kp_a=1;
Ki_a=Kp_a/10;


biasEstimate=[0;0;0];
nvector=[1;0;0];
evector=[0;-1;0];
dvector=[0;0;-1];



Ro=eye(3);
angleX = deg2rad(30);
angleY = deg2rad(180 );
angleZ = deg2rad(30);
rotX=[1 0 0; 
    0 cos(angleX) -sin(angleX); 
    0 sin(angleX) cos(angleX)];

rotY=[cos(angleY) 0 sin(angleY); 
    0 1 0; 
    -sin(angleY) 0 cos(angleY)];

rotZ=[cos(angleZ) -sin(angleZ) 0; 
    sin(angleZ) cos(angleZ) 0; 
    0 0 1];

Ro=rotX*rotY*rotZ;

initNVector=Ro*nvector;
initEVector=Ro*evector;
initDVector=Ro*dvector;

R=Ro;


for i=1:numSteps,              
    gyroInputWithBias=gyroInput+biasTerms;
    wmeas_a=rcross(accelReading)*(R'*accelInertial);
    
    gyroInputWithFeedback = gyroInputWithBias - biasEstimate  + Kp_a*wmeas_a;
    bdot=-Ki_a*wmeas_a;
    biasEstimate = biasEstimate + bdot;
    
    theta(i)=asin(-R(1,3));
    phi(i)=atan2(R(2,3),R(3,3)); % atan2() accounts for which quadrant angle should be
    psy(i)=atan2(R(1,2),R(1,1));

    R=R*Rexp(gyroInputWithFeedback)

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
title('Closed Loop Using Accelerometer Data w/ Bias')
