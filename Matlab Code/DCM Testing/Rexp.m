function R_exp = Rexp(w)
% function R_exp = Rexp(w)
%
% returns the exponential Rodrigues parameter form of the integration that
% keeps R on SO(3). See Park and Chung paper.
%
wnorm = norm(w);
rx = rcross([w(1); w(2); w(3)]);
s = sinc(wnorm/2/pi);  % sinc in matlab has an additional pi term, do not do this in C
c = cos(wnorm/2);
R_exp = [1 0 0;0 1 0;0 0 1] + s*c*rx + s*s/2*rx*rx;