clear all
close all 
clc

%% Connect via serial
pic32 = serialport("COM3", 115200);
configureTerminator(pic32,"CR/LF");
% configureCallback(pic32,"terminator",@serialDat2array)


i=1;
while(1)
    dat(i) = read(pic32);
%     [R]=serialDat2array(dat);
%     disp(dat)
    flush(pic32)
    i=i+1;
end