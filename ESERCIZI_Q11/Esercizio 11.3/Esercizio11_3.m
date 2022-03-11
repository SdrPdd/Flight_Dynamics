% Angolo di 'flottaggio' dell'equilibratore ed effetto del trim tab

clear all; close all; clc;

disp('Moto del velivolo a 3 gradi di libertà');
disp('Risoluzione del problema di trim ad una data altitudine e velocità di volo');

%% Dichiarazione delle variabili globali
global g...                        %Accelerazione di gravità 
       xEG_0 zEG_0 V0 q0 gamma0... %Condizioni iniziali
       rho0 ...                    %Densità dell'aria all'altitudine h = (-zEG_0)
       myAC                        %Oggetto 'Velivolo'

%% Definizione della classe DSVAircraft e dell'oggetto 'Velivolo'
aircraftDataFileName = 'DSV_Aircraft_data.txt';

%Definizione dell'oggetto 'Velivolo'
myAC = DSVAircraft(aircraftDataFileName);

if (myAC.err == -1)
    disp('Terminazione.')
else
    disp(['File ',aircraftDataFileName,' letto correttamente.']);   

    % Costanti e condizioni iniziali
    g = 9.81; %Accelerazione di gravità espressa [m/s^2]
    xEG_0 = 0; %[m]
    zEG_0 = -4000; %Altitudine espressa [m]
    V0 = 257.0; %Velocità di volo espressa [m/s];
    q0 = convangvel(0.000,'deg/s','rad/s'); %Velocità angolare di beccheggio [rad/s]
    gamma0 = convang(0.000,'deg','rad'); %Angolo di rampa [rad]
    [air_Temp0,sound_speed0,air_pressure0,rho0] = atmosisa(-zEG_0);
 
    %% Processo di minimizzazione della funzione di costo
    %Valore di tentativo iniziale per il design vector
    x0 = [0;    %Valore di tentativo iniziale per alpha0 [rad]
          0;    %Valore di tentativo iniziale per delta_e0 [rad]
          0;    %Valore di tentativo iniziale per delta_s0 [rad]
          0.5]; %Valore di tentativo iniziale per delta_T0
      
    %Assegnazione del vincolo a delta_s0
    Aeq = zeros(4,4);
    Aeq(3,3) = 1;
    beq = zeros(4,1);
    delta_s0 = -1.500; %[deg]
    beq(3,1) = convang(delta_s0,'deg','rad');
    
    %Limiti
    lb = [convang(-15,'deg','rad'),... %Valore minimo per alpha
          convang(-20,'deg','rad'),... %Valore minimo per delta_e_0
          convang(-5,'deg','rad'),...  %Valore minimo per delta_s_0
          0.2];                        %Valore minimo per delta_T_0
    ub = [convang(15,'deg','rad'),...  %Valore massimo per alpha
          convang(13,'deg','rad'),...  %Valore massimo per delta_e_0
          convang(5,'deg','rad'),...   %Valore massimo per delta_s_0
          1.0];                        %Valore massimo per delta_T_0

    %0pzioni di ricerca del minimo
    options = optimset('tolfun',1e-9,'Algorithm','interior-point');

    %Chiamata alla funzione 'fmincon'
    [x,fval] = fmincon(@costLongEquilibriumStaticStickFixed,...
                       x0,...                                     
                       [],[],Aeq,beq,...                       
                       lb,ub,...                               
                       @myNonLinearConstraint,...               
                       options);                                  
               
    alpha0_rad = x(1);
    alpha0_deg = convang(alpha0_rad,'rad','deg');
    theta0_rad = alpha0_rad - myAC.mu_x + gamma0;
    theta0_deg = convang(theta0_rad,'rad','deg');
    delta_e0_rad = x(2);
    delta_e0_deg = convang(delta_e0_rad,'rad','deg');
    delta_s0_rad = x(3);
    delta_s0_deg = convang(delta_s0_rad,'rad','deg');
    delta_T0 = x(4);
    
    disp( '')
    disp('Condizione di trim:')
    disp(['Velocità ' num2str(V0) ' m/s'])
    disp(['Angolo d''attacco ' num2str(alpha0_deg) ' deg'])
    disp(['Angolo di deflessione dell''elevatore ' num2str(delta_e0_deg) ' deg'])
    disp(['Angolo di deflessione dello stabilizzatore ' num2str(delta_s0_deg) ' deg'])
    disp(['Grado di ammissione della manetta ' num2str(delta_T0)])

end
 
%% Assegnazione delle leggi temporali dei comandi di volo
global t_sim_first...
       t_fin...
       delta_e...
       delta_s...
       delta_T
   
t_fin = 30; %Tempo finale di simulazione [s] 
t_sim_first = 1; %Tempo di simulazione per la prima fase di volo [s]

delta_e = @(t) interp1([0,t_sim_first],[delta_e0_rad,delta_e0_rad],t,'linear');
delta_s = @(t) interp1([0,t_fin],[delta_s0_rad,delta_s0_rad],t,'linear');
delta_T = @(t) interp1([0,t_fin],[delta_T0,delta_T0],t,'linear');

error = @(delta_tab) mismatch(delta_tab,alpha0_rad,theta0_rad,delta_e0_rad);
delta_t_guessvalue = convang(0,'deg','rad');
options = optimset('Tolfun', 1e-9);
delta_tab_star = fsolve(error,delta_t_guessvalue,options);
disp(['Angolo di deflessione dell''aletta tab tale per cui l''angolo di deflessione dell''equilibratore uguaglia il valore di trim ' num2str(convang(delta_tab_star,'rad','deg')) ' deg'])


global delta_tab
delta_tab = @(t) interp1([0, t_sim_first,  t_sim_first + 1,  t_fin],...
                         [0  0, delta_tab_star,  delta_tab_star],...
                         t,'linear');

%% Prima fase di volo: volo a comandi bloccati
% Integrazione delle equazioni del moto a 3-DoF
state0_first = [V0,alpha0_rad,q0,xEG_0,zEG_0,theta0_rad];

%Integrazione del sistema di equazioni differenziali
options = odeset('RelTol', 1e-9,'AbsTol', 1e-9*ones(1,6));
[vTime1,mState1] = ode45(@eqLongDynamicStickFixed,[0 t_sim_first],state0_first,options);

mState1_dot = zeros(length(vTime1),6);
for i = 1:length(vTime1)
    
    mState1_dot(i,:) = eqLongDynamicStickFixed(vTime1(i),mState1(i,:));
    
end

%% Seconda fase di volo: volo a comandi liberi
% Integrazione delle equazioni del moto a (3+1)DoF

%Per l'intera durata della prima fase di volo i comandi sono assunti
%bloccati. Durante la prima fase di volo, dunque, la variazione nel tempo 
%di \deltae risulta essere nulla. Tale valore nullo costituisce, inoltre, 
%una delle condizioni iniziali per la successiva fase di volo.
delta_e0_dot_rad = convangvel(0,'deg/s','rad/s'); 
state0_second = [mState1(end,1),mState1(end,2),mState1(end,3),...
                 mState1(end,4),mState1(end,5),mState1(end,6),...
                 delta_e0_dot_rad,delta_e0_rad];

%Integrazione del sistema di equazioni differenziali
options = odeset('Mass',@MassStickFree,'RelTol', 1e-9,'AbsTol', 1e-9*ones(1,8));
[vTime2,mState2] = ode45(@eqLongDynamicStickFree,[t_sim_first t_fin],state0_second,options);

mState2_dot = zeros(length(vTime2),8);
for i = 1:length(vTime2)
    
    M = MassStickFree(vTime2(i),mState2(i,:));
    mState2_dot(i,:) = M\eqLongDynamicStickFree(vTime2(i),mState2(i,:));
    
end

%Allocazione in memoria delle storie temporali delle due fasi di volo
mState1 = [mState1,zeros(length(vTime1),1),delta_e0_rad*ones(length(vTime1),1)];
mState_fin = [mState1;mState2(2:end,1:end)];
mState1_dot = [mState1_dot,zeros(length(vTime1),1),zeros(length(vTime1),1)];
mState_fin_dot = [mState1_dot;mState2_dot(2:end,1:end)];
vTime_fin = [vTime1;vTime2(2:end)];

%% Grafica
%Leggi temporali delle variabili di stato, del momento aerodinamico di 
%cerniera e dei fattori di carico
vVel = mState_fin(:,1);
vDelta_V = vVel - V0;
vAlpha_rad = mState_fin(:,2);
vAlpha_deg = convang(vAlpha_rad,'rad','deg');
vDelta_alpha_rad = vAlpha_rad - alpha0_rad;
vDelta_alpha_deg = convang(vDelta_alpha_rad,'rad','deg');
v_q_rad = mState_fin(:,3);
v_q_deg = convangvel(v_q_rad,'rad/s','deg/s');
vDelta_q_rad = v_q_rad - q0;
vDelta_q_deg = convangvel(vDelta_q_rad,'rad/s','deg/s');
vXe = mState_fin(:,4);
vZe = mState_fin(:,5);
vDelta_h = -(vZe - zEG_0);
vTheta_rad = mState_fin(:,6);
vTheta_deg = convang(vTheta_rad,'rad','deg');
vAlphaB_rad = vAlpha_rad - myAC.mu_x;
vAlphaB_deg = convang(vAlphaB_rad,'rad','deg');
vGamma_rad = vTheta_rad - vAlphaB_rad;
vGamma_deg = convang(vGamma_rad,'rad','deg');
vDelta_e_rad = mState_fin(:,8);
vDelta_e_deg = convang(vDelta_e_rad,'rad','deg');
vDelta_e_dot_rad = mState_fin(:,7);
vDelta_e_dot_deg = convangvel(vDelta_e_dot_rad,'rad/s','deg/s');

vVel_dot = mState_fin_dot(:,1);
vAlpha_dot_rad = mState_fin_dot(:,2);
v_q_dot_rad = mState_fin_dot(:,3);
vTheta_dot_rad = mState_fin_dot(:,6);
vGamma_dot_rad = vTheta_dot_rad - vAlpha_dot_rad;
vDelta_e_dot_rad = mState_fin_dot(:,8);
vAlpha_h_rad = (1 - myAC.DepsDalpha)*vAlphaB_rad - myAC.eps_0 +...
                delta_s(vTime_fin) + myAC.mu_x;
vAlpha_h_dot_rad = (1 - myAC.DepsDalpha)*vAlpha_dot_rad;
v_fxa = -(sin(vGamma_rad) + (vVel_dot/g));
v_fza = cos(vGamma_rad) + (vVel_dot.*vGamma_dot_rad/g);


delta_e_float = mState_fin(end,8);
disp(['Angolo di flottaggio ' num2str(convang(delta_e_float,'rad','deg')) ' deg'])

figure(1)
subplot 311
plot(vTime_fin,vXe,'-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
xlabel('$t (s)$','interpreter','latex','fontsize',11);
ylim([-1000 8000])
ylabel('$x_{EG} (m)$','interpreter','latex','fontsize',11)
subplot 312
plot(vTime_fin,vDelta_h,'-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
xlabel('$t (s)$','interpreter','latex','fontsize',11);
ylim([-1000 1000])
ylabel('$\Delta h (m)$','interpreter','latex','fontsize',11)
subplot 313
plot(vTime_fin,vDelta_V,'-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
xlabel('$t (s)$','interpreter','latex','fontsize',11);
ylim([-10 20])
ylabel('$\Delta V (m/s)$','interpreter','latex','fontsize',11)

figure(2)
subplot 311
plot(vTime_fin,vDelta_alpha_deg,'-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylim([-4 1]) 
ylabel('$\Delta \alpha (deg)$','interpreter','latex','fontsize',11);
subplot 312
plot(vTime_fin,vDelta_q_deg,'-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
xlabel('$t (s)$','interpreter','latex','fontsize',11);
ylim([-10 5]) 
ylabel('$\Delta q (deg/s)$','interpreter','latex','fontsize',11);
subplot 313
plot(vTime_fin,vTheta_deg,'-.','LineWidth',1.5);
hold on;
plot(vTime_fin,vGamma_deg,':','color',[0.4660, 0.6740, 0.1880],'LineWidth',1.5);
grid on
lgd = legend('$\theta(t)$','$\gamma(t)$');
lgd.Interpreter = 'latex'; 
lgd.FontSize = 11;
xlim([0 5])
xlabel('$t (s)$','interpreter','latex','fontsize',11);
ylim([-5 5]) 
ylabel('$(deg/s)$','interpreter','latex','fontsize',11);

figure(3)
subplot 211
plot(vTime_fin,vDelta_e_deg,'-.','LineWidth',1.5);
grid on
xlim([0 5])
ylim([-2.5 3]) 
ylabel('$\delta_{e} (deg)$','interpreter','latex','fontsize',11);
subplot 212
plot(vTime_fin,vDelta_e_dot_deg,'-.','LineWidth',1.5);
grid on
xlim([0 5])
ylim([-200 250]) 
ylabel('$\dot{\delta}_{e} (deg/s)$','interpreter','latex','fontsize',11);
 
figure(4)
subplot 311
plot(vTime_fin,convang(v_q_dot_rad,'rad','deg'),'-.','LineWidth',1.5);
grid on
xlim([2.5 t_fin])
xlabel('$t (s)$','interpreter','latex','fontsize',11);
ylim([-100 30]) 
ylabel('$\dot{q} (deg/s^2)$','interpreter','latex','fontsize',11);
subplot 312
plot(vTime_fin,v_fza,'-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylim([-1 2]) 
ylabel('$f_{z_{A}}$','interpreter','latex','fontsize',11);
subplot 313
plot(vTime_fin,v_fxa,'-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
xlabel('$t (s)$','interpreter','latex','fontsize',11);
ylim([-1 1]) 
ylabel('$f_{x_{A}}$','interpreter','latex','fontsize',11);


