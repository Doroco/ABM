clear all
clc
TmL = 1;
TmR = 1;
Power = 3.14;%Power[W];
Va_rated = 12;%rated Voltage[V]
Ia_rated = 0.470;%rated Current[A]
Wm_rated = 5100*2*pi/60;%rated Angular Velocity[rad/s]
Te_rated = Power/Wm_rated;%rated Torque[Nm]

Jm = 5.4846*10^(-7);%Moment of Inertia [kg*m^2]
b = 2.58984 * 10^(-5);%Coefficient of Viscous Friction [Nms/rad] 
Kt = 0.0195;%Torque Constant [Nm/A]
Ke = 0.0195;%Back E.M.F. Constant [V*s/rad]
R = 6;%Resistance [Ohm]
La = 0.005;%Inductance [H]
n = 49;

Wcc = 2*pi*500;%[rad/s]
Kpc = La * Wcc;
Kic = R * Wcc*1.3;
Kac = 1/Kpc;

t = 0.0002;
Wcs = 2*pi*50;%[rad/s]
Kps = Jm * Wcs/Kt;
Kis = Jm*Wcs^2/(Kt);
Kas = 1/(2.8*Kps);

1 - exp(-1);

Rw = 0.155/2;%[m]
dm = 0.24;%[m]
d = 0.15;
L = 0.1655;
M = 8;%[kg]
J = (1/12)*(0.32^2+0.4^2)*M + M*0.15^2;
