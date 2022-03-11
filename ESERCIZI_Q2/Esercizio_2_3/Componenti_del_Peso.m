clear all 
close all
clc

%% COMPONENTI DELLA FORZA PESO IN ASSI VELIVOLO
% Calcolare le componenti della forza peso del velivolo con seguenti
% caratteristiche: 
% m = 1200 kg  (psi, teta, phi) = (20 deg, 10 deg, 0 deg)

% DATI
m = 1200;       %[kg]
g = 9.81;       %[m/s^2]
% Angoli di Eulero
vEulerAngles = convang([20,10,0],'deg','rad');
psi   = vEulerAngles(1);
teta = vEulerAngles(2);
phi   = vEulerAngles(3);

%Forza peso
W = m*g;        %[N]

%Vettore forza peso Assi Terra
vWe = [0; 0; W]; %vettore colonna

%Matrice di trasformazione da Assi Terra ad Assi Velivolo
Tbe = angle2dcm(psi, teta, phi, 'ZYX');

%Vettore forza peso Assi Velivolo
vWb = Tbe*vWe

%% RAPPRESENTAZIONE DELLE COMPONENTI DEL PESO
% Riprendo gli script dell'Esercizio 1

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
vEulerAngles = convang([20,10,0],'deg','rad');

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

%% Disegno Vettore Forza Peso
hold on
scale_weight = 0.0001;
weightVecMag = scale_weight*W;
quiver3( ...
    vXYZe(1),vXYZe(2),vXYZe(3), ...
    0, 0, weightVecMag, ...
    'AutoScale', 'off', 'Color', [0 0 0], 'LineWidth', 2.5 ...
);

%% Vector W_XB * i_B
Teb = Tbe';
% application point along z_B
pWZB_B = scale_weight.*[0;0;vWb(3)];
pWZB_E = vXYZe' + Teb*pWZB_B;
% Vector W_XB * i_B (body-components)
vWeight_XB_B = scale_weight.*[vWb(1);0;0];
% Vector W_XB * i_B (Earth-components)
vWeight_XB_E = Teb*vWeight_XB_B;
quiver3( ...
    pWZB_E(1), pWZB_E(2), pWZB_E(3), ...
    vWeight_XB_E(1), vWeight_XB_E(2), vWeight_XB_E(3), ...
    'AutoScale', 'off', 'Color', [0 0 0], 'LineWidth', 2.0, ...
    'MaxHeadSize', 4.0 ...
);

%% Plot linee tratteggiate
% Per evidenziare la posizione della Forza Peso, utilizziamo la funzione
% plotPoint3DHelperLines che consente di visionare (tramite linee
% tratteggiate) la posizione del baricentro
p_ax=[2, 2, -2]
hold on;
helperLinesOptions.lineColor = 'k';
helperLinesOptions.lineWidth = 1.5;
helperLinesOptions.lineStyle = ':';
plotPoint3DHelperLines(h_fig1, p_ax, helperLinesOptions);
