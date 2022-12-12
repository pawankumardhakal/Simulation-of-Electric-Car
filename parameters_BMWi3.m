%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all;
% Battery - Li-ion/8mod*12cells
Cn_bat = 60;  %Ah
Vn_bat = 353; % V
E_bat = 22; % kWh, 18.8 kWh net capacity. Other models of the BMWi3 have higher capacity batteries.
Imax = 409; % A, https://wiki.aalto.fi/download/attachments/91692283/high_voltage_batteries_of_bmw_vehicles.pdf?version=1&modificationDate=1398446470505&api=v2
% On some simulations this was increased to 900 A  explore the capabilities of the
% control scheme
Pmax_bat = 125; %kW, (assumption, same as motor)
Initial_SOC = 75;   % [percentage]
Bat_response = 10; % [sec]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% DC/DC Converter - Values designed for the appplication, not part of BMWi3 data
%The data suggests this car model is fed directly from the battery, but to
%match the proposed topology, one can be introduced, with a DC voltage of
%at least the maximum achieved by the battery

Vdc = 600; %V

%For the same reason, there is no data about a dc/dc converter interfacing
%with the battery, so the design available from ESHEV will be used

fsw_boost = 20e3; %Hz
L_boost = 1e-3; %H
R_Lboost = L_boost*250; %ohm
C_boost = 5*475*1e-6; %F
R_Cboost = 1.8e-3; %ohm

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%  Boost converter controller parameters
% Method thought in the Energy Storage subject
% Current controller params
% Zero-pole cancellation
%BW_i =    100;
BW_i =    300;  %[Hz] Closed-loop bandwidth for IL (Average Model)
%BW_i = 200; %Bandwidth for IL (Switching Model)
Kp_idc = 2*pi*BW_i*L_boost;
Ki_idc = 2*pi*BW_i*R_Lboost;
% Voltage controller params
% Free selection of parameters
%BW_v = 20;
BW_v =  30;  %[Hz] Closed-loop bandwidth for Vo (Average Model)
%BW_v = 10; %bandwidth for Vo (Switching Model)
Mp = .2;                    %[pu] Overshoot
Wn_v = 2*pi*BW_v;           %[rad/s] Natural frequency
xi = cos(atan(-pi/log(Mp)));
Kp_vdc = 2*xi*Wn_v*C_boost;
Ki_vdc = Wn_v^2*C_boost;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% DC/AC Inverter

%Modulation scheme->sine-triangle with zero harmonic injection (assumption)

fsw_inv = 10e3; %Hz % From "Adopting MOSFET Multilevel Inverters to Improve the Partial Load Efficiency of Electric Vehicles"; F. Chang, O. Ilina, O. Hegazi, L. Voss, M. Lienkamp;
fsw_inv1 = 20e3;%average model switching frequency
fsw_inv2 = 10e3;%swithing model switching frequency
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%fsw_inv = 1e3; %for WLTP3 speed porfile test, average model
%Tsim = 1e-4; %sample time for WLTP3 speed profile test, average model
Tsim = 5e-6; % Average model sample time
Tsim_2 = 1e-5; % Switching model sample time


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Machine Parameters
% Machine - hybrid synchronous

% Performance
% V range = 250 - 400 V
Tmax_mach = 250; %Nm % https://www.trader-media.ie/templates/bmw2015/forms/brochure_pdfs/E229404_BMWi.pdf
Pmax_mach = 125*1e3; %W
wb_mach = 4800*pi/30; %rad/s, @75kW
nmax_mach = 11400; %rpm % https://evobsession.com/bmw-i3-details/
wmax_mach = nmax_mach*pi/30; %rad/s

% Electrical data and lumped parmeters

In_mach = 240*sqrt(2); %A; 
Imotor_max = 400; % A phase;
Vn_mach = 400; %V
p = 6; % pole pairs %http://hybridfordonscentrum.se/wp-content/uploads/2014/05/20140404_BMW.pdf
Ld_mach = 0.0858*1e-3; %H https://ssl.lvl3.on24.com/event/13/49/76/5/rt/1/documents/resourceList1488311786346/webinar_power_point.pdf
Lq_mach = 0.24*1e-3; %H
R_phase_mach = 0.005225; %ohm
J_mach = 0.0666; %kgm2
% Xd_mach = 0.615; %ohm
% Xq_mach = 1.72; %ohm
ke_mach = 41.67; %V/krpm
% kt_mach = 0.523; %Nm/A
% n_noload_mach = 8950; %rpm
flux_mach = 200/(6*4800*pi/30); %Wb
deltaL_mach = Ld_mach - Lq_mach; %H

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Motor Current Controller
% d axis
%Bw_c = 150*2*pi;   % Bandwidth of current controller
Bw_c = 500*2*pi;   % Bandwidth of current controller
Mp_c = 0.02;       % Maximum overshoot 2%
xi_c = cos(atan(-pi/log(Mp_c)));
Kp_id = 2*xi_c*Bw_c*Ld_mach-R_phase_mach;   % Proportional gain id controller
Ki_id = Bw_c^2 * Ld_mach;     %  Integrator gain id controller
% q axis
Kp_iq = 2*xi_c*Bw_c*Lq_mach-R_phase_mach;   % Proportional gain iq controller
Ki_iq = Bw_c^2 * Lq_mach;      % Integrator gain iq controller

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% MTPA Values of id and iq
% Values used for look-up table 
I_step = 0.1;    % [A]
I_demand = -Imotor_max-1e-3 : I_step : Imotor_max-1e-3;      
beta = acos((-flux_mach + sqrt(flux_mach ^2 + 8* (Ld_mach-Lq_mach)^2 * I_demand.^2)) ./ (4*(Ld_mach-Lq_mach)*I_demand));
id = I_demand .* cos(beta);
iq = I_demand .* sin(beta);
T_dq = (3/2)*(p)*(flux_mach*iq+(Ld_mach-Lq_mach)*iq.*id); % Mechanical torque 
figure(1)
plot(I_demand,T_dq);
xlim([-500,500]);
grid on;
xlabel('I-demand (A)');
ylabel('Torque-dq (Nm)');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%torque speed characteristics
Tmax = 250; %%Nm
Pmax = 125; %Kw
Wbase = 4800; %%rpm
Wmax = 11400; %%rpm
w = 0:1:Wmax;
T = zeros(1,length(w));

%%use of logical indexing to access the index of the vectors%%
wbase = Pmax*1e3/Tmax; %%rad/s
wbase_rpm = wbase*30/pi;
T(w<=wbase_rpm) = Tmax;
T(w>wbase_rpm) = Pmax*1e3./(w(w>wbase_rpm)*pi/30);
P = w.*T*(2*pi/60);
figure(2)
plot(w,T,'-'); grid on;
ylim([100,350]);
xlabel ('Speed (RPM)');
ylabel ('Torque (Nm)');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Wheels - 155/70 R19 84 Q

r_wheel = (15+2*70/(100*175))*0.0254/2; %m

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Mechanical transmission

%single speed transmission
vmax = 150; %km/h
nmax_wheel = vmax*1e3/(60*2*pi*r_wheel); %rpm
k_gear = nmax_mach/nmax_wheel;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Vehicle body % rear wheel drive

mass = 1270; %kg
%50:50 weight distribution
CoG_x = 0;
CoG_y = 0;
CoG_z = 18.5*0.0254; %m % http://www.mybmwi3.com/forum/viewtopic.php?t=1751
wheelbase = 2570; 
A_front = 2.38; % m^2
C_drag = 0.29; %https://www.press.bmwgroup.com/global/article/attachment/T0143924EN/222601
max_load = 1620; %kg
wheel_span = 2.570; %m
rho=1.25;% air density, kg/m3
fr=0.013; % rolling resistant coefficient
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Performance to be expected

% 100km, NEDC, combined -> 13.1 s
% Range (NEDC) -> 359 km
% top speed -> 150 km/h
% Acceleration 0-100 km/h -> 7.2 s
% Acceleration 80-120 km/h -> 4.9 s

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%





