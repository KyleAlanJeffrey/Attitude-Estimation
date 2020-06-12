close all
clear all
clc

%% Import Experimental Sinc


sincData = csvread('sincData.csv');
x=sincData(:,1);
y=sincData(:,2);

plot(x,y)
hold on
plot(x,sin(x)./x);
legend('Experimental', 'Actual')
xlim([-5 5])