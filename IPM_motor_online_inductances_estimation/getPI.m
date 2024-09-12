function [kp, ki] = getPI(process, phim, wgc, Ts)

[magP, phaP] = bode(process, wgc);
DeltaK = 1/magP;
DeltaPhi = phim - pi - phaP*pi/180;

kp = DeltaK*cos(DeltaPhi);
ki = -wgc*DeltaK*sin(DeltaPhi);

if (nargin == 4)
    ki = ki*Ts;
end

end

