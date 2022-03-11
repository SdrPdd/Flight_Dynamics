close all
clear all
clc

%% ORIENTAMENTO DI UN VELIVOLO IN ASSI TERRA - Esercizio 2.1
%Si produca una rappresentazione del modello tridimensionale di un velivolo

%% Carico modello 3D del velivolo
% Scelgo un modello in formato .stl da importare
% Utilizzo la funzione da libreria loadAircraftSTL e read_stl in cui
% bisogna lasciare tutto invariato

scale_factor = 1;
[V, F, C] = loadAircraftSTL('aircraft_pa24.stl', scale_factor);

% Salvo in formato .mat le matrici e conseguentemente il velivolo
shape.V = V; shape.F = F; shape.C = C;
save('aircraft_pa24.mat', 'shape');

%% Creo la scena con la giusta orientazione degli assi (zE positivo verso il basso)
h_fig1 = figure(1);
grid on;
hold on;
light('Position',[1 0 -2],'Style','local');

% Inversione degli assi per averli nel verso desiderato
set(gca,'XDir','reverse');
set(gca,'ZDir','reverse');

%% Mostro il velivolo a video

%Per la visualizzazione esterna del velivolo usiamo il comando patch
p = patch('faces', shape.F, 'vertices' ,shape.V);
set(p, 'facec', [0 1 1]);                                                  %Setto come rosso il colore delle facce       
set(p, 'EdgeColor','none');                                                %Nessun colore per gli edge
theView = [-125 30];                                                       %Angoli che definicono la posizione dell'osservatore
view(theView);
axis equal;
lighting phong                                                             %Modalità luce sui soldi
hold on

%% Plot degli assi
xMax = 1.8*max(abs(shape.V(:,1)));
yMax = 1.8*max(abs(shape.V(:,2)));
zMax = 0.5*xMax;
quiver3( ...
    0,0,0, ...
    xMax,0,0, ...
    'r','linewidth',2.5 ...
); hold on;
quiver3( ...
    0,0,0, ...
    0,yMax,0, ...
    'g','linewidth',2.5 ...
); hold on;
quiver3( ...
    0,0,0, ...
    0,0,zMax, ...
    'b','linewidth',2.5 ...
); hold on;
