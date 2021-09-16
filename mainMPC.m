%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This software simulates an MPC-based battery charge control algorithm for
% a cylindrical cell A123 26650 2.5 Ah (LPF chemistry)
%
% Copyright (c) 2021 by Aloisio Kawakita de Souza of the
% University of Colorado Colorado Springs (UCCS). This work is licensed
% under a MIT license. It is provided "as is", without express or implied
% warranty, for educational and informational purposes only.
%
% This file is provided as a supplement to: 
%
% [1] Kawakita de Souza, A. (2020). Advanced Predictive Control Strategies for 
%  Lithium-Ion Battery Management Using a Coupled Electro-Thermal Model 
%  [Master thesis, University of Colorado, Colorado Springs].ProQuest Dissertations Publishing.
%
% [2] A. K. de Souza, G. Plett and M. S. Trimboli, "Lithium-Ion Battery Charging Control Using 
% a Coupled Electro-Thermal Model and Model Predictive Control," 2020 IEEE Applied Power 
% Electronics Conference and Exposition (APEC), 2020, pp. 3534-3539, 
% doi: 10.1109/APEC39645.2020.9124431.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 



clear;close all;
tic
addpath('./functions');
load('A123model.mat');

Nsim = 1000;  % Duration of simulation
deltaT = 1;   % Sampling time
time = 1:Nsim; % Time vector

SOC_ref = 95;  % SOC target  [%]
T0 = 25;       % Initial temperature [C]
Tf = 25;       % Cooling temperature [C]

% Plant Initial Conditions
SOC0_plant = 0.1;           % Plant's real SOC
ir0_plant = 0;              % Initial RC pair current
h0_plant = 0;               % Initial hysteresis state
Ts0_plant = T0;             % Plant's initial Ts
Tc0_plant = T0;             % Plant's initial Tc

% MPC Tuning variables
mpcData.Np = 5;         % Prediction horizon
mpcData.Nc = 3;         % Control horizon
mpcData.Tf = Tf;
mpcData.DUk_1 = 0;
mpcData.model = model;
mpcData.deltaT = deltaT;

% MPC constraints
mpcData.const.u =  20;       % Max charging current [A]
mpcData.const.v_max = 3.6;   % Max voltage [V]
mpcData.const.tc_max = 50;   % Max core temperature [C]
mpcData.const.z_max = SOC_ref/100;  % Max SOC
mpcData.const.du_max = 15;         
 
% Storing variables
voltage_store = zeros(Nsim,1);
x_store = zeros(5,Nsim);
u_store = zeros(1,Nsim);
u_ref_store = zeros(1,Nsim);

% Plant initialization
xp0 = [SOC0_plant; ir0_plant; h0_plant; Ts0_plant; Tc0_plant];
xp = xp0;
uk=0;
[voltage, xp] = iterModel(xp,uk,Tf,model,deltaT);
x_store(:,1) = xp;
voltage_store(1) = voltage;
u_store(1) = uk;

hwait = waitbar(0,'Charging...');
for k = 2:Nsim
    
    % Plant
    [voltage, xp] = iterModel(xp,uk,Tf,model,deltaT);

    % MPC iteration
    mpcData.k = k;
    [uk, mpcData] = iterMPC(SOC_ref,xp,mpcData);

    
    % Storing
    x_store(:,k) = xp;  % [SOC; irc; h; Ts; Tc]
    voltage_store(k) = voltage;
    u_store(k) = uk;
    Ru_store(k) = mpcData.Ru;
    

   % Update progress
   if mod(k,10)==0, waitbar(k/Nsim,hwait); end;
end

toc
close(hwait);

plotFigures;
