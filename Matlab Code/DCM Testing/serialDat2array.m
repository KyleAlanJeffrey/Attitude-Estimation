function [R] = serialDat2array(dat)
%SERIALDAT2ARRAY Summary of this function goes here
%   Detailed explanation goes here
j=1;
R=eye(3);
for i=1:3
    tok = strtok(dat,',');
    if i == 3
       tok=strtok(dat,'/LF') ;
    end
    
    R(j,i)=str2double(tok);
    
    if i==3
        j = j+1;
    end
end
end

