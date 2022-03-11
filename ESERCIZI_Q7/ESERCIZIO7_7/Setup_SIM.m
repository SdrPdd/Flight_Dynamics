%% Simulazione del moto a 3-DOf a partire da condizioni di trim
clear all; close all; clc;

disp('Moto del velivolo a 3 gradi di libertà');
disp('Risoluzione del problema di trim ad una data altitudine e velocità di volo');

%% Dichiarazione delle variabili globali
global g...                  %Accelerazione di gravità 
       zEG_0 V0 q0 gamma0... %Condizioni iniziali
       rho0 ...              %Densità dell'aria all'altitudine h = (-zEG_0)
       myAC                  %Oggetto 'Velivolo'

%% Ricerca delle condizioni di trim
aircraftDataFileName = 'DSV_Aircraft_data.txt';

%  Definizione dell'oggetto 'Velivolo'
myAC = DSVAircraft(aircraftDataFileName);

if (myAC.err == -1)
    disp('Terminazione.')
else
    disp(['File ',aircraftDataFileName,' letto correttamente.']);
end 
     
%  Costanti e condizioni iniziali
g = 9.81;                               %Accelerazione di gravità [m/s^2]
xEG_0 = 0;                              %[m]
zEG_0 = -4000;                          %Altitudine [m]
V0 = 320.0;                             %Velocità di volo [m/s];
%V=[240,260,280,300,320,340]; CAMBIARE VELOCITA' OGNI VOLTA
q0 = convangvel(0.000,'deg/s','rad/s'); %Velocità angolare di beccheggio
gamma0 = convang(0.000,'deg','rad');    %Angolo di rampa
[air_Temp0,sound_speed0,air_pressure0,rho0] = atmosisa(-zEG_0);

%% SIMULINK

% Si ricavano le condizioni di trim importando i dati iniziali 
% nel modello simulink 'test_steady_state_manager7_7.slx'

% Tramite l'utilizzo dello "Steady State Manager" sono state ricavate le
% condizioni di trim relative a ciascun valore di velocità precedentemente
% fissati 

% Nel file 'postprocess7_7.m' vengono riportati gli output per
% ottenere il plot dei dati

