close all
clear all
clc

%% POSIZIONAMENTO DI UN VELIVOLO NELLO SPAZIO
% Mediante l'utilizzo della funzione loadAircaftMAT posso richiamere
% dirattemente la geometria del velivo

%% Creo la scena con la giusta orientazione degli assi (zE positivo verso il basso)
h_fig1 = figure(1);
grid on;
hold on;
light('Position',[1 0 -2],'Style','local');

% Inversione degli assi per averli nel verso desiderato
set(gca,'XDir','reverse');
set(gca,'ZDir','reverse');

%% Carico la geometria del velivolo
% Utilizzando la funzione fornita, carico la geometria del velivolo nel
% formato .mat
shapeScaleFactor = 1.0;
shape = loadAircraftMAT('aircraft_pa24.mat', shapeScaleFactor);

%% Posiziono il velivolo nello spazio

% Posizione di Assi Terra del Baricentro
vXYZe= [2,2,-2];

% Terna angoli di Eulero[psi, theta, phi]
vEulerAngles = convang([30,20,10],'deg','rad');

theView = [105 15];                                                        %Posizione dell'osservatore

% Assi Velivolo utilizzando la funzione plotBodyE
bodyAxesOptions.show = true;
bodyAxesOptions.magX = 2.0;
bodyAxesOptions.magY = 2.0;
bodyAxesOptions.magZ = 2.0;
bodyAxesOptions.lineWidth = 2.5;
plotBodyE(h_fig1, shape, vXYZe, vEulerAngles, bodyAxesOptions, theView);

%% Plot Assi Terra
% Oltre agli assi Body, plottiamo anche gli Assi Terra nella stessa figura
% utilizzando la funzione plotEarthAxes

hold on;
xMax = max([abs(vXYZe(1)),5]);
yMax = max([abs(vXYZe(2)),5]);
zMax = 0.3*xMax;                                                           % max([abs(max(vXYZe(1))),0.18*xMax]);
vXYZ0 = [0,0,0];
vExtent = [xMax,yMax,zMax];
plotEarthAxes(h_fig1, vXYZ0, vExtent);

%% Plot linee tratteggiate
% Per evidenziare la posizione del baricentro, utilizziamo la funzione
% plotPoint3DHelperLines che consente di visionare (tramite linee
% tratteggiate) la posizione del baricentro

hold on;
helperLinesOptions.lineColor = 'k';
helperLinesOptions.lineWidth = 1.5;
helperLinesOptions.lineStyle = ':';
plotPoint3DHelperLines(h_fig1, vXYZe, helperLinesOptions);

