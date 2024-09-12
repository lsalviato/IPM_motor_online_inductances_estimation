function [kp, ki] = getPiCur(wgc, phim, R, L, tauD, Ts)
% get current PI kp and ki gain

s = tf('s');
sysMot = 1/(R + s*L);
sysDelay = exp(-s*tauD);
% model delay as linear function
% sysDelayApp = 1/(1+s*tauD);

[magP, phaP] = bode(sysMot*sysDelay, wgc);
DeltaK = 1/magP;
DeltaPhi = phim - pi - phaP*pi/180;
kp = DeltaK*cos(DeltaPhi);
ki = -wgc*DeltaK*sin(DeltaPhi);

% get discrete parameters if Ts is passed as a parameter
if (nargin == 6)
    ki=ki*Ts;
end

end
