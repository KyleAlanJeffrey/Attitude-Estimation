clear all
close all
clc

%% THIS FILE COMPARES TRIAD AND CLOSED LOOP ATTITUDE FILTERING OFF THE PIC32 IMPLIMENTATION
%% Read Data in CSV
DAT = csvread('data/TRIAD_ClosedLoop_TUMBLE.csv');

%% Seperate DCMS
j = 1; %R index
l = 1; %R_triad index
k = 1; %DAT index
for i=1:length(DAT)/6
    R(j:j+3,:)=DAT(k:k+3,:);
    k=k+3;
    j=j+3;
    R_triad(l:l+3,:)=DAT(k:k+3,:);
    k=k+3;
    l=l+3;
end

%% Compare DCM's 
[error, std] = compareDCM(R,R_triad)

