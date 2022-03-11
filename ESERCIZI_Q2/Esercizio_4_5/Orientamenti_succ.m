clear all
close all
clc

%% Orientamenti successivi di un velivolo
% Rappresentare una successione di posizioni e orientamenti nello spazio
% per un velivolo che passa da ali orizzontali a phi = -60 deg

%Avvalendoci del supporto della funzione: plotTrajectoryAndBodyE
% plotTrajectoryAndBodyE(h_fig1, shape, mXYZe, mEulerAngles, options);

%% POSIZIONAMENTO DI UN VELIVOLO NELLO SPAZIO
% Mediante l'utilizzo della funzione loadAircaftMAT posso richiamere
% dirattemente la geometria del velivo

%% Creo la scena con la giusta orientazione degli assi (zE positivo verso il basso)
h_fig1 = figure(1);
grid on;
hold on;
light('Position',[1 0 -4],'Style','local');

% Inversione degli assi per averli nel verso desiderato
set(gca,'XDir','reverse');
set(gca,'ZDir','reverse');
daspect([1 1 1]);

%% Carico la geometria del velivolo
% Utilizzando la funzione fornita, carico la geometria del velivolo nel
% formato .mat
shapeScaleFactor = 200.0;
shape = loadAircraftMAT('aircraft_pa24.mat', shapeScaleFactor);

%% Prepare CoG coordinates and Euler angles
steps = 200;
speed = 100.0; % m/s
radius = 4000.0; % m
omegaTurn = speed/radius; % 1/s
timeFinal = 50; % s
vTime = transpose(linspace(0,timeFinal,steps));
% Posisions in Earth axes
vXe = radius - radius.*cos(omegaTurn.*vTime);
vYe = radius.*sin(omegaTurn.*vTime);
vZe = -1000.*ones(steps,1);
mXYZe = [vXe,vYe,vZe];
% psi, theta, phi -> 'ZYX'
vT = transpose(linspace(0,pi/3,steps));
vPsi = (pi/2)*(1. - 1.7*vTime./timeFinal);
vTheta = 0.0.*ones(steps,1);
vPhi = -vT;
mEulerAngles = [vPsi,vTheta,vPhi];

%% Settings
% General settings
options.samples = [1,40,80,120,160];
options.theView = [105 15];

% body axes settings
options.bodyAxes.show = true;
options.bodyAxes.magX = 1.5*shapeScaleFactor;
options.bodyAxes.magY = 2.0*shapeScaleFactor;
options.bodyAxes.magZ = 2.0*shapeScaleFactor;
options.bodyAxes.lineWidth = 2.5;

% helper lines
options.helperLines.show = true;
options.helperLines.lineStyle = ':';
options.helperLines.lineColor = 'k';
options.helperLines.lineWidth = 1.5;

% trajectory
options.trajectory.show = true;
options.trajectory.lineStyle = '-';
options.trajectory.lineColor = 'k';
options.trajectory.lineWidth = 1.5;

%% Plot body and trajectory
plotTrajectoryAndBodyE(h_fig1, shape, mXYZe, mEulerAngles, options);

%% Plot Earth axes
hold on;
xMax = max([max(abs(mXYZe(:,1))),5]);
yMax = max([max(abs(mXYZe(:,2))),5]);
zMax = 0.05*xMax; % max([abs(max(vXYZe(1))),0.18*xMax]);
vXYZ0 = [0,0,0];
vExtent = [xMax,yMax,zMax];
plotEarthAxes(h_fig1, vXYZ0, vExtent);
xlabel('x_E (m)'); ylabel('y_E (m)'); zlabel('z_E (m)')
