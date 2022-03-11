clc;clear all; close all;
%% CINEMATICA DELL'EVOLUZIONE DI LOOPING PERFETTO IN SIMULINK

%% Condizioni iniziali
% Supponiamo un velivolo inizialmente in volo livellato, con fusoliera
% orizzontale e prua diretta verso il nord

% Angoli di Eulero in deg
psi0=0; theta0=0; phi0=0;

% Posizione iniziale negli assi
xe0=0; ye0=0; ze0=0;

qman= 1;
u0= 100;
v0=0;
w0=0;

% Attraverso l'utilizzo della funzione angle2quat è possibile ottenere le
% componenti del quaternione {q0 qX qY qZ}'
Q0= angle2quat(psi0,theta0,phi0);

% Tempo di simulazione
t0=0; tfin= 12;
tinm=0.2*tfin;
tdurm=min(2*pi/qman,0.95*tfin)+0.85;% durata nominale

%% p q r a partire da valori puntuali
vtimeprofilep = [0  1]*tdurm;
vprofilep = [0 0];
vtimeprofileq =       [0 tinm+(tdurm)*[0 1/30 1/10  1/5 0.7  0.87 0.93    0.95  0.99 1] tfin];
vprofileq =           [0 qman*[0 1/4  3/4    1   1     1  0.85    0.7 0.01 0] 0];
vtimeprofiler = [0  1]*tdurm;
vprofiler = [0  0];
%% u v w a partire da valori puntuali

vtimeprofileu = tfin* [0 1/30 1/10 1/5 0.4 0.95 0.97 1];
vprofileu  =     u0*[1 .96 .93 .9 .9 .9 .9 .9];
vtimeprofilev =[0 1]*tdurm;
vprofilev =[1 1];
vtimeprofilew =[0  1]*tdurm;
vprofilew =[1  1];
%% Definizione dei segnali
signalu=[vtimeprofileu',vprofileu'];
signalv=[vtimeprofilev',vprofilev'];
signalw=[vtimeprofilew',vprofilew'];
signalp=[vtimeprofilep',vprofilep'];
signalq=[vtimeprofileq',vprofileq'];
signalr=[vtimeprofiler',vprofiler'];

