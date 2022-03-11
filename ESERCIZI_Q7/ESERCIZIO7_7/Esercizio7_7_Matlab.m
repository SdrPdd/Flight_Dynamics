%% Ricerca di condizioni di trim per un moto a 3-DoF al variare della
%  velocità iniziale
%  HP. delta_s0 = -1°
clear all; close all; clc;

disp('Moto del velivolo a 3 gradi di libertà');
disp('Risoluzione del problema di trim ad una data altitudine e velocità di volo');

%% Dichiarazione delle variabili globali
global g...                   %Accelerazione di gravità 
       zEG_0 V0 q0 gamma0 ... %Condizioni iniziali
       rho0 ...               %Densità dell'aria all'altitudine h = (-zEG_0)
       myAC                   %Oggetto 'Velivolo'

%% Ricerca delle condizioni di trim
aircraftDataFileName = 'DSV_Aircraft_data.txt';

%Definizione dell'oggetto 'Velivolo'
myAC = DSVAircraft(aircraftDataFileName);

if (myAC.err == -1)
    disp('Terminazione.')
else
    disp(['File ',aircraftDataFileName,' letto correttamente.']);   

    % Costanti e condizioni iniziali
    g = 9.81;           %Accelerazione di gravità [m/s^2]
    xEG_0 = 0;          %[m]
    zEG_0 = -4000;      %Altitudine [m]
    q0 = convangvel(0.000,'deg/s','rad/s'); %Velocità angolare di beccheggio
    gamma0 = convang(0.000,'deg','rad');    %Angolo di rampa [rad]
    [air_Temp0,sound_speed0,air_pressure0,rho0] = atmosisa(-zEG_0);
    NVel = 20;
    vVel0 = linspace(220.0,350.0,NVel);     %Velocità di volo [m/s]
 
    %% Processo di minimizzazione della funzione di costo
    %  Valore di tentativo iniziale per il design vector
    x0 = [0;    %Valore di tentativo iniziale per alpha_0 [rad]
          0;    %Valore di tentativo iniziale per delta_e0 [rad]
          0;    %Valore di tentativo iniziale per delta_s0 [rad]
          0.5]; %Valore di tentativo iniziale per delta_T0
      
    %  Limiti
    lb = [convang(-15,'deg','rad'),... %Valore minimo per alpha
          convang(-20,'deg','rad'),... %Valore minimo per delta_e
          convang(-5,'deg','rad'),...  %Valore minimo per delta_s
          0.2];                        %Valore minimo per delta_T
    ub = [convang(15,'deg','rad'),...  %Valore massimo per alpha
          convang(13,'deg','rad'),...  %Valore massimo per delta_e
          convang(5,'deg','rad'),...   %Valore massimo per delta_s
          1.0];                        %Valore massimo per delta_T
      
    %  0pzioni di ricerca del minimo
    options = optimset('tolfun',1e-9,'Algorithm','interior-point'); %tolleranza
         
    vDelta_s0 = convang([0.000,-1.000],'deg','rad');
    %Vettore dei valori di \delta_s0 per i quali si intende
    %valutare il punto di minimo della funzione di costo
    %al variare della velocità
    
    %Inizializzazione dei vettori contenenti le variabili di output                  
    alpha0_deg = zeros(NVel,length(vDelta_s0));       
    delta_e0_deg = zeros(NVel,length(vDelta_s0));        
    delta_s0_deg = zeros(NVel,length(vDelta_s0));        
    delta_T0  = zeros(NVel,length(vDelta_s0));
       
    for j = 1:length(vDelta_s0)
        
        Aeq = zeros(4);
        Aeq(3,3) = 1;
        beq = zeros(4,1);
        beq(3,1) = vDelta_s0(j);
        
        for i = 1:NVel
            
            %Chiamata alla funzione 'fmincon'
            [x,fval] = fmincon(@(x) costLongEquilibriumStaticStickFixedVel(x,vVel0(i)),...     
                               x0,...                                     
                               [],[],Aeq,beq,... 
                               lb, ub,...                                  
                               @myNonLinearConstraint,...
                               options);                                  

            alpha0_deg(i,j) = convang(x(1),'rad','deg');
            delta_e0_deg(i,j) = convang(x(2),'rad','deg');
            delta_s0_deg(i,j) = convang(x(3),'rad','deg');
            delta_T0(i,j) = x(4);
            
        end
        
    end
    
    %% Grafica
    figure(1)
    subplot 311
    plot(vVel0,alpha0_deg(:,2),'b-o','LineWidth',1.5,'markersize',2.5);
    hold on;
    plot(vVel0,alpha0_deg(:,1),'b:o','LineWidth',1.5,'markersize',2.5);
    grid on
    lgd = legend('$\delta_{s,0} = -1 deg$','$\delta_{s,0} = 0 deg$');
    lgd.Interpreter = 'latex'; 
    lgd.FontSize = 11;
    xlim([vVel0(1)-10 vVel0(end)+10])
    ylim([0 3.5])
    ylabel('$\alpha_0(deg)$','interpreter','latex','fontsize',11)
    subplot 312
    plot(vVel0,delta_e0_deg(:,2),'b-o','LineWidth',1.5,'markersize',2.5);
    hold on;
    plot(vVel0,delta_e0_deg(:,1),'b:o','LineWidth',1.5,'markersize',2.5);
    grid on
    lgd = legend('$\delta_{s,0} = -1 deg$','$\delta_{s,0} = 0 deg$');
    lgd.Interpreter = 'latex'; 
    lgd.FontSize = 11;
    xlim([vVel0(1)-10 vVel0(end)+10])
    ylim([-7 -0.5])
    ylabel('$\delta_{e0}(deg)$','interpreter','latex','fontsize',11)
    subplot 313
    plot(vVel0,delta_T0(:,2),'b-o','LineWidth',1.5,'markersize',2.5);
    hold on;
    plot(vVel0,delta_T0(:,1),'b:o','LineWidth',1.5,'markersize',2.5);
    grid on
    lgd = legend('$\delta_{s,0} = -1 deg$','$\delta_{s,0} = 0 deg$');    
    lgd.Interpreter = 'latex'; 
    lgd.FontSize = 11;
    xlim([vVel0(1)-10 vVel0(end)+10])
    xlabel('$V_0 (m/s)$','interpreter','latex','fontsize',11);
    ylim([0 1])
    ylabel('$\delta_{T0}$','interpreter','latex','fontsize',11)
    
end