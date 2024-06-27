% PMSM FOC course
% Lab1. Current control on DC motor
% Author: Daniel Montesinos-Miracle, Oriol Subirats-Rillo
% October 2023

clearvars; close all; clc;

%% Vehicle parameters
Vehicle.mv = 150;       % Vehicle mass [kg]
Vehicle.Cd = 0.85;      % Drag coeficient [-]
Vehicle.Af = 0.4;       % Frontal area [m2]
Vehicle.fr = 0.025;     % Rolling resistance coeficient [-]
Vehicle.rw = 0.06;      % Tyre effective rolling radius [m]
Vehicle.kred = 10;       % Gearbox ratio [-]

%% Environment parameters
Environment.rho = 1.22521;  % Air deny constant [kg/m3]
Environment.g = 9.81;       % Gravity constant [m/s2]
Environment.vair = 0;                   % Air speed [m/s]
Environment.grade = 0;                  % Grade [%]
Environment.alpha = atan(Environment.grade/100);    % Angle [rad]

%% Motor parameters. Maxon RE 65, 250 W, 48 V
Motor.Electrical.Ra = 0.365;                     % Winding resistance [Ohm]
Motor.Electrical.La = 0.161e-3;                  % Winding inductane [H]
Motor.Electrical.kmotor = 1/77.8*60/(2*pi);      % Motor constant [V·s/rad]
Motor.Electrical.Umr = 48;                       % Motor rated voltage [V]
Motor.Electrical.Imr = 6.8;                      % Motor rated current [A]
Motor.Thermal.R_thcase_amb = 1.3;             % Case to ambient thermal resistance [K/W]
Motor.Thermal.tau_thcase = 1060;              % Thermal time constant motor case [s]
Motor.Thermal.R_thwind_case = 1.85;           % Winding to case thermal resistance [K/W]
Motor.Thermal.tau_thwind = 123;               % Thermal time constant motor winding [s]
Motor.Efficiency = 0.88;                      % Motor maximum efficiency [-]

%% Battery parameters
Battery.Electrical.Ubat = 48;  % Battery voltage

%% Speed controller parameters
% IMC
Control.Speed.tr_v = 0.3;                   % Rise time [s]
Control.Speed.alpha_v = log(9)/Control.Speed.tr_v;
Control.Speed.Kp_v = Control.Speed.alpha_v*Vehicle.mv;
Control.Speed.Ki_v = 0;

%% Current controller parameters
% IMC
Control.Current.tr_i = 100e-3;                   % Rise time [s]
Control.Current.alpha_i = log(9)/Control.Current.tr_i;
Control.Current.Kp_i = Control.Current.alpha_i*Motor.Electrical.La;
Control.Current.Ki_i = Control.Current.alpha_i*Motor.Electrical.Ra;

open DCmotor;
sim DCmotor;