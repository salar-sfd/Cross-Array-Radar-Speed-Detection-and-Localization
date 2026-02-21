clc, clear, close all;

%% Signal Properties
h = 6;
theta0 = 90 - atand(37.5/h);
fc = 2e9;
dmax = 2;
Trec = 0.1;

%% Tx Array Design

[Tx.d, Tx.w] = Generate_Tx_Array(theta0, fc, dmax, 'opt');
Calculate_Directivity(Tx, fc, theta0, 1);

%% Rx Array Design

[Rx.d, Rx.w] = Generate_Rx_Array(fc, dmax, 'opt');
Calculate_Directivity(Rx, fc, theta0, 1);

%% Plotting Final System

Plot_Cross_Array(Tx.d, Tx.w, Rx.d, Rx.w, dmax);

%% Simulation
N_cars = 7;
Tsim = 5;
Tpause = 0;

Highway_Sim(N_cars, h, Tsim, Trec, Tpause)
