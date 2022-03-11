%% esercizio 3.5.2
clc;clear all; close all;
%% initial conditions
psi0=0; theta0=0; phi0=0;
xe0=0; ye0=0; ze0=0;

pman = convangvel(4.0,'deg/s','rad/s'); % rad/s
qman = convangvel(2.0,'deg/s','rad/s'); % rad/s
u0= 380/3.6;
v0=0;
w0=0;
Q0= angle2quat(psi0,theta0,phi0);

%% time span
nsteps= 100;
t0=0; tfin= 130;
tinm= 0.1*tfin;

tdurm= 105;

%% p q r a partire da valori puntuali
vtimeprofilep = [0 tinm+tdurm*[0 1/30 1/10 1/5 0.8 0.9 1] tfin];
vprofilep = [0 pman*[0 1/40 3/4 1 1 0 0] 0];
vtimeprofileq= [0 tinm+tdurm*[0 1/30 1/10 1/5 0.8 0.9 1] tfin];
vprofileq = [0 qman*[0 1/40 3/4 1 1 0 0] 0];
vtimeprofiler = [0  1]*tfin;
vprofiler = [0  0];
%% u v w a partire da valori puntuali

vtimeprofileu = [0 tinm+tdurm*[0 1/30 1/10 1/5 0.8 1] tfin];
vprofileu  =   [u0 u0*[1 0.8 0.7 1 1.1 1] u0];
vtimeprofilev =[0 1]*tfin;
vprofilev =[1 1];
vtimeprofilew =[0  1]*tfin;
vprofilew =[1  1];
%% definizione dei segnali
tt=linspace(0,tfin,20000);
signalu= [ tt', interp1(vtimeprofileu',vprofileu',tt)'];
signalv=[ tt', interp1(vtimeprofilev',vprofilev',tt)'];
signalw=[ tt', interp1(vtimeprofilew',vprofilew',tt)'];
signalp=[ tt', interp1(vtimeprofilep',vprofilep',tt)'];
signalq=[ tt', interp1(vtimeprofileq',vprofileq',tt)'];
signalr=[ tt', interp1(vtimeprofiler',vprofiler',tt)'];


