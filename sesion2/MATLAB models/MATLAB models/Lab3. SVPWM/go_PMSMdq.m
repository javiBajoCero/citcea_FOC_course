% PMSM FOC course
% Lab1. Current control on DC motor
% Author: Daniel Montesinos-Miracle, Oriol Subirats-Rillo
% October 2023

clearvars; close all; clc;

%% Vehicle parameters
Vehicle.mv = 150;       % Vehicle mass [kg]
Vehicle.Cd = 0.85;      % Drag coeficient [-]
Vehicle.Af = 0.4;       % Frontal area [m2]
Vehicle.fr = 0.025;      % Rolling resistance coeficient [-]
Vehicle.rw = 0.06;      % Tyre effective rolling radius [m]
Vehicle.kred = 14;      % Gearbox ratio [-]

%% Environment parameters
Environment.rho = 1.22521;                          % Air deny constant [kg/m3]
Environment.g = 9.81;                               % Gravity constant [m/s2]
Environment.vair = 0;                               % Air speed [m/s]
Environment.grade = 0;                             % Grade [%]
Environment.alpha = atan(Environment.grade/100);    % Angle [rad]

%% Battery parameters
Battery.Electrical.Ubat = 36;   % Battery voltage

%% Motor parameters 
Motor.Electrical.Rs = 0.0071;               % Motor resistance [Ohm]
Motor.Electrical.Ld = 2.4133e-4;            % Motor Ld inductance [H]
Motor.Electrical.Lq = 2.5733e-4;            % Motor Lq inductance [H}  
Motor.Electrical.lambda = 0.0068;           % Rotor flux [V·s/rad]
Motor.Electrical.n = 14;                    % Motor poles [-]
Motor.Electrical.pp = Motor.Electrical.n/2; % Pole pairs [-]
Motor.Electrical.Umax = Battery.Electrical.Ubat/sqrt(3);    % Maximum phase to neutral peak voltage [V]
Motor.Electrical.Imax = 150*sqrt(2);                         % Maximum peak phase current [A]

%% Speed controller parameters
% IMC, PI controller
Control.Speed.tr_v = 0.3;                                 % Rise time [s]
Control.Speed.alpha_v = log(9)/Control.Speed.tr_v;      % Alpha parameter for IMC
Control.Speed.Kp_v = Control.Speed.alpha_v*Vehicle.mv;  % Kp constant
Control.Speed.Ki_v = 0;                                 % Ki constant

%% Current controllers parameters
% IMC
Control.Current.tr_id = 5e-3;                                         % Rise time [s]
Control.Current.alpha_id = log(9)/Control.Current.tr_id;                % Alpha parameter for IMC
Control.Current.Kp_id = Control.Current.alpha_id*Motor.Electrical.Ld;   % Kp constant
Control.Current.Ki_id = Control.Current.alpha_id*Motor.Electrical.Rs;   % Ki constant
Control.Current.Kaw_id = 50;                                             % Kaw constant

Control.Current.tr_iq = 5e-3;                                         % Rise time [s]
Control.Current.alpha_iq = log(9)/Control.Current.tr_iq;                % Alpha parameter for IMC
Control.Current.Kp_iq = Control.Current.alpha_iq*Motor.Electrical.Lq;   % Kp constant
Control.Current.Ki_iq = Control.Current.alpha_iq*Motor.Electrical.Rs;   % Ki constant
Control.Current.Kaw_iq = 50;                                             % Kaw constant

Motor.Electrical.Isc = Motor.Electrical.lambda / Motor.Electrical.Ld;   % Short-circuit current (A)

Motor.Electrical.alpha = 1/(3*Motor.Electrical.pp*(Motor.Electrical.Lq-Motor.Electrical.Ld));   % alpha = 1/(3*pp*(Lq-Ld));
Motor.Electrical.beta = Motor.Electrical.lambda/(Motor.Electrical.Lq-Motor.Electrical.Ld);      % beta = lambda/(Lq-Ld);
Motor.Electrical.mu = Motor.Electrical.beta - Motor.Electrical.lambda/Motor.Electrical.Ld;      % mu = beta - lambda/Ld;

%% FW controller parameters

% Flux-Weakening controller with 1st ordre response
Control.FluxWeakening.KFW = 0.8;                             % Relationship between the maximum available voltage and the applicable voltage [pu]
Control.FluxWeakening.tsV = 50e-3;                          % Flux.weakening closed-loop settling time (s)
Control.FluxWeakening.Is_WP = Motor.Electrical.Imax;         % Current magnitude working point (A)
Control.FluxWeakening.Is_WP = 50;         % Current magnitude working point (A)
Control.FluxWeakening.gamma_WP = 135*pi/180;                 % Current angle working point (rad)

Control.FluxWeakening.Umax_FW = Motor.Electrical.Umax*Control.FluxWeakening.KFW;      % RMS voltage according to the security factor (V)

% Electrical speed at the Working Point (rad/s)
Control.FluxWeakening.A = -Motor.Electrical.Rs * Control.FluxWeakening.Is_WP * sin(Control.FluxWeakening.gamma_WP) * (Control.FluxWeakening.Is_WP * cos(Control.FluxWeakening.gamma_WP) * (Motor.Electrical.Ld-Motor.Electrical.Lq) + Motor.Electrical.lambda) / (Motor.Electrical.Lq^2 * Control.FluxWeakening.Is_WP^2 * sin(Control.FluxWeakening.gamma_WP)^2 + (Motor.Electrical.Ld * Control.FluxWeakening.Is_WP * cos(Control.FluxWeakening.gamma_WP) + Motor.Electrical.lambda)^2);
Control.FluxWeakening.B = Motor.Electrical.Rs^2 * Control.FluxWeakening.Is_WP^2 * sin(Control.FluxWeakening.gamma_WP)^2 * (Control.FluxWeakening.Is_WP * cos(Control.FluxWeakening.gamma_WP) * (Motor.Electrical.Ld-Motor.Electrical.Lq) + Motor.Electrical.lambda)^2 / ((Motor.Electrical.Lq^2 * Control.FluxWeakening.Is_WP^2 * sin(Control.FluxWeakening.gamma_WP)^2 + (Motor.Electrical.Ld * Control.FluxWeakening.Is_WP * cos(Control.FluxWeakening.gamma_WP) + Motor.Electrical.lambda)^2))^2;
Control.FluxWeakening.C = ((Motor.Electrical.Lq^2 * Control.FluxWeakening.Is_WP^2 * sin(Control.FluxWeakening.gamma_WP)^2 + (Motor.Electrical.Ld * Control.FluxWeakening.Is_WP * cos(Control.FluxWeakening.gamma_WP) + Motor.Electrical.lambda)^2)) * (Motor.Electrical.Rs^2 * Control.FluxWeakening.Is_WP^2 - Motor.Electrical.Umax^2) / ((Motor.Electrical.Lq^2 * Control.FluxWeakening.Is_WP^2 * sin(Control.FluxWeakening.gamma_WP)^2 + (Motor.Electrical.Ld * Control.FluxWeakening.Is_WP * cos(Control.FluxWeakening.gamma_WP) + Motor.Electrical.lambda)^2))^2;

Control.FluxWeakening.we_WP = Control.FluxWeakening.A + sqrt(Control.FluxWeakening.B-Control.FluxWeakening.C);              % rad/s
Control.FluxWeakening.wm_WP = Control.FluxWeakening.we_WP/Motor.Electrical.pp * (30/pi);         % rpm

% Integral parameter at the WP
Control.FluxWeakening.A = 2*Control.FluxWeakening.Is_WP*(Motor.Electrical.Rs*Control.FluxWeakening.we_WP*Control.FluxWeakening.tsV*Control.FluxWeakening.Is_WP*(Motor.Electrical.Ld-Motor.Electrical.Lq)*cos(Control.FluxWeakening.gamma_WP)^2);
Control.FluxWeakening.B = Control.FluxWeakening.Is_WP*cos(Control.FluxWeakening.gamma_WP)*(5*(Motor.Electrical.Ld-Motor.Electrical.Lq)*(Motor.Electrical.Rs-Control.FluxWeakening.tsV*(Motor.Electrical.Ld+Motor.Electrical.Lq)*Control.FluxWeakening.we_WP^2/5)*Control.FluxWeakening.Is_WP*sin(Control.FluxWeakening.gamma_WP) + Motor.Electrical.lambda*Control.FluxWeakening.we_WP*(Motor.Electrical.Rs*Control.FluxWeakening.tsV - 5*Motor.Electrical.Lq));
Control.FluxWeakening.C = Control.FluxWeakening.Is_WP*Control.FluxWeakening.we_WP*(sin(Control.FluxWeakening.gamma_WP)*Motor.Electrical.Ld*Control.FluxWeakening.we_WP*Control.FluxWeakening.tsV*Motor.Electrical.lambda + Control.FluxWeakening.Is_WP*(Motor.Electrical.Rs*(Motor.Electrical.Ld-Motor.Electrical.Lq)*Control.FluxWeakening.tsV+5*Motor.Electrical.Ld*Motor.Electrical.Lq));

Control.FluxWeakening.Kp_V = 0;
Control.FluxWeakening.Ki_V = -5*Motor.Electrical.Umax/(Control.FluxWeakening.A + Control.FluxWeakening.B - Control.FluxWeakening.C);
Control.FluxWeakening.Kaw_V = 2.5;

open PMSMdq;
sim PMSMdq;