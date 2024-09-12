function [kp,ki] = getPiVel(wgc, phim, B, J, tau_gc_cur, Ts)
% get velocity PI kp and ki gain

s = tf('s');
sysMot= 1/(B + s*J);
syscur= 1/(1 + s*tau_gc_cur);

[magP, phaP] = bode(sysMot*syscur,wgc);
DeltaK = 1/magP;
DeltaPhi = phim - pi - phaP*pi/180;

kp = DeltaK*cos(DeltaPhi);
ki = -wgc*DeltaK*sin(DeltaPhi);

if (nargin == 6)
    ki=ki*Ts;
end

end

