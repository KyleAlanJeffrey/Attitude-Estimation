clear all
close all
clc

%% Import
mag = csvread('magTumble.csv');

scatter3(mag(:,1),mag(:,2),mag(:,3));
xlabel('x-axis')
ylabel('y-axis')
zlabel('z-axis')

axis equal