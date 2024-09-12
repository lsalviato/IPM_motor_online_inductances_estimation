clear all; close all;

%% electrical parameters
%select the motor to simulate
motor=2; %motor "A" = 1  ;  motor "B"=2

if(motor==1)
    mot.p = 4;
    mot.R  = 1.55;     %[Ohm] stator resistance
    mot.kTau = 0.207;  %1.5*mot.p*mot.lammg; %[Vs] torque constant
    mot.lammg = mot.kTau/mot.p/1.5;    %[Vs] 
    mot.Ld = 5.1e-3;    %[H]  
    mot.Lq = 9.6e-3;    %[H]

    %characteristics parameters
    mot.IN = 5; % (A) motor rated currents
    mot.UN = 20; % (V) motor rated voltage
    mot.WN = 3e3; % (rpm) motor rated speed
    mot.tauN = mot.IN*mot.kTau; % (Nm) motor rated torque 1.0350

    %mechanical
    mot.B = 0.05*mot.tauN/(mot.WN*2*pi/60); % (N*m*s/rad) Viscous friction
    mot.J = 46.1e-6;
    
    inv.Ubus   = 40; 
else
    mot.p = 4;
    mot.R = 1.45;     %[Ohm] stator resistance
    mot.kTau = 0.172;  %1.5*mot.p*mot.lammg; %[Vs] torque constant
    mot.lammg = mot.kTau/mot.p/1.5;    %[Vs] 
    mot.Ld = 6e-3;    %[H]
    mot.Lq = 18e-3;    %[H]

    %characteristics parameters
    mot.IN = 25; % (A) motor rated currents
    mot.UN = 24; % (V) motor rated voltage
    mot.WN = 3e3; % (rpm) motor rated speed
    mot.tauN = mot.IN*mot.kTau; % (Nm) motor rated torque 4.3000

    %mechanical
    mot.B = 0.05*mot.tauN/(mot.WN*2*pi/60); % (N*m*s/rad) Viscous friction
    mot.J = 99.6e-6;  
    
    inv.Ubus = 80; 
end

%% lookup tables
idVec = -mot.IN:1:mot.IN;    %[A]
iqVec = -(mot.IN*2):1:(mot.IN*2);  %[A]  

lamdVec = mot.Ld*idVec + mot.lammg;
lamqVec = mot.Lq*iqVec;

[lamdMap,lamqMap] = meshgrid(lamdVec,lamqVec);
[idMap,iqMap] = meshgrid(idVec,iqVec);

%% INVERTER NON IDEALITIES
inv.Fs = 20e3; % (Hz) Sampling frequency
inv.Ts = 1/inv.Fs; % (s) Time period of the sampling
inv_Ts = inv.Ts;
inv.tauD = 1.5*inv.Ts;

inv.Fpwm = 20e3; % (Hz) Switching frequency
inv.Tpwm = 1/inv.Fpwm; % (s) Time period of the switching

inv.nid.Vsw = 0.08; % (V) Switch voltage drop
inv.nid.Vf = 1.5; % (V) Freewheeling diode voltage drop
inv.nid.Vcm = (inv.nid.Vf+inv.nid.Vsw)/2;
inv.nid.Vdm = (inv.nid.Vf-inv.nid.Vsw)/2;

inv.nid.DT = 1.5e-6; % (s) dead time
inv.nid.toff = 0.34e-6; % (s) turn off time of the switch
inv.nid.ton = 0.2e-6; % (s) turn on time of the switch
inv.nid.ttot = inv.nid.DT + inv.nid.ton - inv.nid.toff;

%% PI cur
PI.cur.wgc = 200*(2*pi); % (rad/s) Control bandwidth
PI.cur.phim = 80*(pi/180); % (rad) Phase margin
PI.cur.Ts = inv.Tpwm;
inv.tauD = 1.5*inv.Tpwm;

%continuous gains
%[PI.cur.kpd,PI.cur.kid] = getPiCur(PI.cur.wgc,PI.cur.phim,mot.R,mot.Ld,inv.tauD);
%[PI.cur.kpq,PI.cur.kiq] = getPiCur(PI.cur.wgc,PI.cur.phim,mot.R,mot.Lq,inv.tauD);

PI.cur.Ld = mot.Ld;
PI.cur.Lq = mot.Lq;
PI.cur.lammg = mot.lammg;
[PI.cur.kpd,PI.cur.kid] = getPiCur(PI.cur.wgc,PI.cur.phim,mot.R,PI.cur.Ld,inv.tauD,PI.cur.Ts);
[PI.cur.kpq,PI.cur.kiq] = getPiCur(PI.cur.wgc,PI.cur.phim,mot.R,PI.cur.Lq,inv.tauD,PI.cur.Ts);

%% PI VEL
PI.vel.fs = 12.5e3;
PI.vel.Ts = 1/PI.vel.fs;
PI.vel.wgc = 10*(2*pi);
PI.vel.phim = 80*pi/180;
[PI.vel.kp, PI.vel.ki] = getPiVel(PI.vel.wgc,PI.vel.phim,mot.B,mot.J,1/PI.cur.wgc,PI.vel.Ts);
PI.vel.kt = PI.vel.ki/PI.vel.kp;

%% MEASURES
%Current measurement acquisition 
meas.cur.adc.FS = 2*mot.IN; % (A) Full scale 

meas.cur.adc.Nb = 12; % ADC number of bit
meas.cur.adc.q = meas.cur.adc.FS/2^meas.cur.adc.Nb; % (A)

%Encoder measurement acquisition
meas.enc.Np = 250; % Number of pulses 250 vs 1000
meas.enc.q = 2*pi/meas.enc.Np;

meas.enc.nTabs = 100; %100 is a good value
meas.enc.Ts = inv.Ts; %sampling time

%% Recursive Least Squares
lambda = 0.9995; %0.9995 default
