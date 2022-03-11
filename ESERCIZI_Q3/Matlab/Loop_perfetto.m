clear all
close all
clc

%% CINEMATICA DELL'EVOLUZIONE DI LOOPING PERFETTO IN MATLAB
% Si scriva un programma che risolve il problema di valori iniziali
% assegnato e disegna i grafici necessari

%% Condizioni iniziali
% Supponiamo un velivolo inizialmente in volo livellato, con fusoliera
% orizzontale e prua diretta verso il nord

% Per applicare la 1.6, ci occorrono gli angoli di Eulero da mettere nella
% 3.61 [3] per ottenere il vettore {q0 qX qY qZ}'

% Angoli di Eulero in deg
psi_0 = 0;     %[deg]
teta_0 = 0;    %[deg]
phi_0 = 0;     %[deg]

% Conversione in rad 
psi_0=convang(psi_0,'deg','rad'); 
teta_0=convang(teta_0,'deg','rad');
phi_0=convang(phi_0,'deg','rad');

% Attraverso l'utilizzo della funzione angle2quat è possibile ottenere le
% componenti del quaternione {q0 qX qY qZ}'
Q_0=angle2quat(psi_0, teta_0, phi_0) 

% Posizione iniziale negli assi
x_0=0;
y_0=0;
z_0=0;
PosE0=[x_0,y_0,z_0];

% Tempo di simulazione [t_0, t_f]
t_0=0;
t_f=2*pi; 
tt=linspace(t_0,t_f,1e3);

%% Quaternione
% Ora per applicare la 1.6 mi occore conoscere le componenti 
% della velocità anngolare {p (t), q(t), r(t)}, 
% dipendenti dal tempo in quanto il sistema evolve.

% Per il looping p (rollio) ed r (imbardata) restano identicamente nulle,
% q (beccheggio) costante non nulla

qmax=1;
p=@(t) 0;
r=@(t) 0;
q=@(t) interp1([0.05*t_f 0.1*t_f 0.2*t_f 0.3*t_f 0.4*t_f 0.5*t_f ...
       0.6*t_f 0.7*t_f 0.8*t_f 0.9*t_f t_f ],...
       [qmax qmax qmax qmax qmax qmax qmax qmax qmax qmax qmax],t,'pchip');

% Possiamo risolvere il secondo membro della 1.6
dQuatdt = @(t,Q) 0.5.*[0,-p(t),-q(t),-r(t);
p(t), 0, r(t),-q(t);
q(t),-r(t), 0, p(t);
r(t), q(t),-p(t), 0]*Q;

% Opzioni di tolleranza numerica
options=odeset('RelTol',1e-9,'AbsTol',1e-9);      
% Equazione ode45
[vtime, vquat] = ode45(dQuatdt, [0 t_f], Q_0, options); 

%% Funzione che restituisce dal quaternone le velocitá angolari

[vpsir,vthetar,vphir]=quat2angle(vquat); %Quaternione angoli in rad eq3.62

% Converto gli angoli in deg
vpsi=convang(vpsir,'rad','deg');         
vtheta=convang(vthetar,'rad','deg');
vphi=convang(vphir,'rad','deg');

%% Traslazione del baricentro

% Al Tempo t=0 in m/s
u0=100;
v0=0;
w0=0; 

V0=sqrt(u0*u0+v0*v0+w0*w0); %modulo della velocitá

u=@(t)interp1([0.05 0.1*t_f 0.2*t_f 0.3*t_f 0.4*t_f 0.5*t_f...
      0.6*t_f 0.7*t_f 0.8*t_f 0.9*t_f t_f],[u0 u0 u0 ...
      u0 u0 u0 u0 u0 u0 u0 u0],t,'pchip');
v=@(t) 0;
w=@(t) 0;

%% Plots

h1=figure(1)

% Plot Angoli di Eulero
subplot 511 
plot(vtime,vpsi,'-','LineWidth',2.0)
hold on
plot(vtime,vtheta,'--','LineWidth',2.0)
plot(vtime,vphi,'-.','LineWidth',2.0)
legend('\psi','\theta','\phi','Location','best')
set(legend,'color','none');
grid minor
%xlabel(’t(s)’);
ylabel('(deg)')
title('Angoli di Eulero')

% 4 componenti del quaternione
subplot 512 %Stabilisco due immagini della stessa figura
plot(vtime,vquat(:,1),'-','LineWidth',2.0)
hold on
plot(vtime,vquat(:,2),'--','LineWidth',2.0)
plot(vtime,vquat(:,3),'-.','LineWidth',2.0)
plot(vtime,vquat(:,4),':','LineWidth',2.0)
grid minor
legend('q_0','q_x','q_y','q_z','Location','best')
set(legend,'color','none');
%xlabel(’t(s)’);
ylabel('');
title('Componenti del Quaternione')

% Velocitá angolari convertite in deg di p,q,r
subplot 513 
plot(vtime, convangvel(p(vtime),'rad/s','deg/s'),'-','LineWidth',2.0)
hold on
plot(vtime, convangvel(q(vtime),'rad/s','deg/s'),'--','LineWidth',2.0)
plot(vtime, convangvel(r(vtime),'rad/s','deg/s'),'-.','LineWidth',2.0)
legend('p(t)','q(t)','r(t)','Location','best')
set(legend,'color','none');
%xlabel(’t (s)’);
ylabel('(deg/s)')
title('Leggi temporali delle componenti della velocitá del velivolo nel BRF')
grid minor;

% Velocitá del baricentro in m/s
subplot 514 
plot(vtime, u(vtime),'-','LineWidth',2.0)
hold on
plot(vtime, v(vtime),'--','LineWidth',2.0)
plot(vtime, w(vtime),'-.','LineWidth',2.0)
legend('u(t)','v(t)','w(t)','Location','best')
set(legend,'color','none');
%xlabel(’t (s)’);
ylabel('(m/s)');
title('Leggi temporali delle componenti della velocitá di traslazione del baricentro')
grid minor;

%% Integrazione delle equazioni della Navigazione noto il quaternione

Quate = @(t) ... %Sto integrando le componenti del Quaternione nel tempo
[interp1(vtime,vquat(:,1),t), ...
interp1(vtime,vquat(:,2),t), ...
interp1(vtime,vquat(:,3),t), ...
interp1(vtime,vquat(:,4),t)];
T_BE = @(Q)quat2dcm(Q); % Matrice di trasformazione da Earth to body axes

% RHS equazioni della navigazione (Eq2.26)
dPosEdt = @(t,PosE) transpose(quat2dcm(Quate(t)))*[u(t);v(t);w(t)];

%% Soluzioni delle equazioni della navigazione
options = odeset('RelTol',1e-9,'AbsTol',1e-9*ones(3,1));
[vtime, vPosE] = ode45(dPosEdt, vtime, PosE0, options);
N = length(vPosE);
vXe = vPosE(:,1); vYe = vPosE(:,2); vZe = z_0 + vPosE(:,3);
subplot 515
plot(vtime,vPosE(:,1),'-','LineWidth',2.0)
hold on
plot(vtime,vPosE(:,2),'--','LineWidth',2.0)
plot(vtime,vPosE(:,3),'-.','LineWidth',2.0)
legend('x_EG(t)','y_EG(t)','z_EG(t)','Location','best')
set(legend,'color','none');
xlabel('t (s)'); ylabel('(m)')
title('Leggi temporali delle componenti della posizione del baricentro')
grid minor;
% prop={'PaperType','PaperSize','PaperPosition','PaperPositionMode','PaperUnits','Units'};
% valori={'<custom>',[18,29],[0,0,18,29],'manual', 'Inches', 'Inches'};
% set(h1, prop, valori);

%% Orientamenti successivi di un velivolo

% Rappresentare una successione di posizioni e orientamenti nello spazio
% per un velivolo

% Avvalendoci del supporto della funzione: plotTrajectoryAndBodyE
% plotTrajectoryAndBodyE(h_fig1, shape, mXYZe, mEulerAngles, options);


%% POSIZIONAMENTO DEL VELIVOLO NELLO SPAZIO
% Mediante l'utilizzo della funzione loadAircaftMAT posso richiamere
% dirattemente la geometria del velivo

%% Creo la scena con la giusta orientazione degli assi (zE positivo verso il basso)
h_fig2 = figure(2);
grid on;
hold on;
light('Position',[1 0 -4],'Style','local');

% Inversione degli assi per averli nel verso desiderato
set(gca,'XDir','reverse');
set(gca,'ZDir','reverse');
daspect([1 1 1]);

%% Trasformazione dal quaternione agli angoli di Eulero
for kk=1:size(vquat,1)
dcm = quat2dcm(vquat(kk,:));
for ir=1:3
for ic=1:3
vDCM(kk,ir,ic) = dcm(ir,ic);
end
end
[vpsi(kk), vtheta(kk), vphi(kk)] = quat2angle(vquat(kk,:));
end

%% Carico la geometria del velivolo
% Utilizzando la funzione fornita, carico la geometria del velivolo nel
% formato .mat
shapeScaleFactor = 20.0;
shape = loadAircraftMAT('aircraft_pa24.mat', shapeScaleFactor);
mXYZe = [vPosE(:,1),vPosE(:,2),vPosE(:,3)+z_0];
mEul = [vpsi,vtheta,vphi];

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
plotTrajectoryAndBodyE(h_fig2, shape, mXYZe, mEul, options);

%% Plot Earth axes
%% Stampa assi Terra
hold on;
xMax = max([max(abs(mXYZe(:,1))),5]);
yMax = max([max(abs(mXYZe(:,2))),5]);
zMax = 0.05*xMax; % max([abs(max(vXYZe(1))),0.18*xMax]);
vXYZ0 = [0,0,0];
vExtent = [xMax,yMax,zMax];
plotEarthAxes(h_fig2, vXYZ0, vExtent);
xlabel('x_E (m)'); ylabel('y_E (m)'); zlabel('z_E (m)')
hold off
%print(h_fig2,'-deps','images\immaginetraiettoria');


