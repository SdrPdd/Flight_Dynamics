clear all
close all
clc

%% CINEMATICA DELL'EVOLUZIONE DI CUBAN EIIGHT IN MATLAB


%% Condizioni iniziali
% Supponiamo un velivolo inizialmente in volo livellato, con fusoliera
% orizzontale e prua diretta verso il nord

% Per applicare la 1.6, ci occorrono gli angoli di Eulero da mettere nella
% 3.61 [vedi Quaderno 3] per ottenere il vettore {q0 qX qY qZ}'

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
t_f=14; 
tt=linspace(t_0,t_f,1e3);

%% Quaternione
% Ora per applicare la 1.6 mi occore conoscere le componenti 
% della velocità anngolare nel tempo {p (t), q(t), r(t)},
% dipendenti dal tempo in quanto il sistema evolve

p_cost=1.278; %valori in rad/s
q_cost=1.305;
r_cost=0;

tp_1 = 3.29 ;  tp_2 = 5.95 ;  dtp12 = tp_2-tp_1; tp_3 = 9.28; tp_4 = 12.0;
v_time_p = [0,   tp_1,   tp_1+dtp12/30, tp_1+dtp12/10, tp_1+0.8*dtp12, tp_1+0.9*dtp12, tp_2, tp_3, tp_3+dtp12/30, tp_3+dtp12/10,  tp_3+0.9*dtp12 , tp_4, t_f ];
v_pt     = [0,    0 ,     p_cost*3/4,         p_cost,         p_cost,         p_cost,           0     0       p_cost*3/4,         p_cost,            p_cost        0      0];
p = @(t) interp1(v_time_p,v_pt,t,'pchip' )  ;          % velocità angolare di rollio in rad/s

tq_1 = 2*pi ;   tq_2 = 6.2;  tq_3 = 12.39;  
v_time_q = [0,   tq_1/30,   tq_1/10,     tq_1/5, tp_1, tp_1+dtp12/30,tp_1+0.76*dtp12, tp_1+0.81*dtp12,    tq_2+tp_1/25, tq_2+tp_1/20,  tp_3 , tp_3+dtp12/30, 11.43 ,      tq_3,    tq_3+dtp12/20,   12.54, t_f];
v_qt     = [0,   q_cost/40,  q_cost*3/4,     q_cost,   q_cost,       0     ,          0,         q_cost/40,         q_cost*3/4,       q_cost,       q_cost,         0,       0   ,       0.7*q_cost,        0.6*q_cost,      0        0];
q = @(t) interp1(v_time_q,v_qt,t,'pchip' );          % velocità angolare di beccheggio in rad/s

r = @(t) 0*sign(exp(t))  ;          % velocità angolare di imbardata in rad/s

r = @(t) ...
    interp1( ...
        [0 t_f/30 t_f/10 t_f/5 0.8*t_f 0.9*t_f t_f], ...
        [0 r_cost/40 r_cost*3/4 r_cost r_cost 0 0], ...
        t, 'pchip' ...
    );
% Possiamo risolvere il secondo membro della 1.6
dQuatdt = @(t,Q) 0.5.*[0,-p(t),-q(t),-r(t);
p(t), 0, r(t),-q(t);
q(t),-r(t), 0, p(t);
r(t), q(t),-p(t), 0]*Q;

%Opzioni di tolleranza numerica
options=odeset('RelTol',1e-9,'AbsTol',1e-9);    
%Equazione ode45
[vtime, vquat] = ode45(dQuatdt, [0 t_f], Q_0, options); 

%% Funzione che restituisce dal quaternone le velocitá angolari
[vpsir,vthetar,vphir]=quat2angle(vquat); %Quaternione angoli in rad 
vpsi=convang(vpsir,'rad','deg');         %Converto gli angoli in deg
vtheta=convang(vthetar,'rad','deg');
vphi=convang(vphir,'rad','deg');

%% Traslazione del baricentro
% al Tempo t=0 in m/s
u0 = 250/3.6 ;   v0 = 0 ;   w0 = 0 ;   % componenti di velocità all'istante 0 in m/s
V0 = sqrt(u0*u0+v0*v0+w0*w0);      % modulo della velocità che supponiamo costante durante la manovra
u = @(x) u0*sign(exp(x))    ; 
%u = @(x) interp1( [ti   t_f/30  t_f/10 t_f/5 0.4*t_f tp_3+0.9*dtp12 0.96*t_f t_f],...
%                   [u0  u0*0.95 u0*0.9 u0*0.85 u0*0.85 u0*0.9 0.95*u0 u0 ], x, 'pchip');   
v = @(x) 0.*sign(exp(x))    ;          
w = @(x) 0.*sign(exp(x))    ;          

%% Plots

h1=figure(1)

%Plot angoli di Eulero
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
subplot 512 
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

%Velocitá angolari convertite in deg di p,q,r
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

%Velocitá del baricentro in m/s
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

% RHS equazioni della navigazione (Eq3.25)
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


%% Orientamenti successivi di un velivolo
% Rappresentare una successione di posizioni e orientamenti nello spazio
% per un velivolo che passa da ali orizzontali a phi = -60 deg

%Avvalendoci del supporto della funzione: plotTrajectoryAndBodyE
% plotTrajectoryAndBodyE(h_fig1, shape, mXYZe, mEulerAngles, options);


%% POSIZIONAMENTO DEL VELIVOLO NELLO SPAZIO
% Mediante l'utilizzo della funzione loadAircaftMAT posso richiamere
% dirattemente la geometria del velivo

%% Creo la scena con la giusta orientazione degli assi 
%(zE positivo verso il basso)

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
options.samples = [1,100,300,500,600];
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

