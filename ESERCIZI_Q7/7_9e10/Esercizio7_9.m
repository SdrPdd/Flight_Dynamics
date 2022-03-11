clear all; clc; close all;

disp('Moto del velivolo a 3-DoF');

%% Variabili globali e parametri del velivolo
global g ... 
       z_0...
       V_0 ...
       q_0 ...
      gamma_0 ...
      rho_0 ...
      myAC ...
      delta_e ...
      delta_s_0 ...
      delta_T_0
aircraftDataFileName = 'DSV_Aircraft_data.txt';

myAC = DSVAircraft(aircraftDataFileName);
if (myAC.err == -1)
    disp(['... terminating.']);
else
    disp(['File ', aircraftDataFileName, ' letto correttamente.']);
    
    %% Condizioni iniziali e altri dati
    z_0     = -4000.;                  % altitudine
    q_0     = 0.;                      % velocità angolare di beccheggio (rad/s)
    gamma_0 = convang(0,'deg','rad');  % angolo di volta (rad)
    g = 9.81;                          % (m/s^2)
    [air_Temp_0, sound_speed_0, air_pressure_0, rho_0] = atmosisa(-z_0);
    
    %% Vettore di tentativo
    %xi0=[alpha_0, delta_e_0, delta_s_0, delta_T_0]
    xi0 = [0; 0; 0; 0.2];
    
    %% Minimizzazione della funzione di costo
    % Aeq, in Aeq*x=beq linear constraint
    Aeq       = zeros(4,4);
    beq       = zeros(4,1);
    Aeq(3,3)  = 1;                              
    delta_s_0 = convang(-1,'deg','rad');
    beq(3,1)  = delta_s_0;                      % fissa delta_s = delta_s_0
    
    % limiti inferiori per [alpha, delta_e, delta_s, delta_T]
    lb = [convang([-15,-20,-5],'deg','rad'),0.2];
    
    % limiti superiori per [alpha, delta_e, delta_s, delta_T]
    ub = [convang([15,13,2],'deg','rad'),1.0];
    
    V=270;
   vec = zeros(length(V),4); %preallocazione
    for i=1:length(V)
        V_0=V(i);
        options = optimset('tolfun',1e-9,'Algorithm','interior-point');
        [xi,fval] = fmincon(@costLongEquilibriumStaticStickFixed, ...
            xi0,[],[],Aeq,beq,lb,ub,@myNonLinearConstraint,options);
        
        vec(i,:)=xi;
    end
    
    %%%%%xi0=[alpha_0, delta_e_0, delta_s_0, delta_T_0]
    
    alpha_0       = xi(1);
    alpha_0_deg   = convang(alpha_0,'rad','deg');
    theta_0       = gamma_0 + alpha_0 + myAC.mu_x;
    theta_0_deg   = convang(theta_0,'rad','deg');
    delta_e_0     = xi(2);
    delta_e_0_deg = convang(delta_e_0,'rad','deg');
    delta_s_0     = xi(3);
    delta_s_0_deg = convang(delta_s_0,'rad','deg');
    delta_T_0     = xi(4);
    
    disp('')
    disp('Condizione di trim:')
    disp(['Velocità V_0= ',num2str(V_0),' m/s'])
    disp(['Angolo d''attacco alpha_0= ',num2str(alpha_0_deg),'°'])
    disp(['Equilibratore delta_e_0= ',num2str(delta_e_0_deg),'°'])
    disp(['Stabilizzatore delta_s_0= ',num2str(delta_s_0_deg),'°'])
    disp(['Manetta delta_T_0= ',num2str(delta_T_0)])
    
    %% Risoluzione delle equazioni del moto e calcolo fattori di carico (CASO A)
    
    t1=1; t2=2.5; t3=4; t_fin=10; D_e=convang(-3.,'deg','rad'); %de.ta_e - delta_e_0

    
    delta_e = @(t) ...
        interp1([0,t1,t2,t3,t_fin],[delta_e_0,delta_e_0,...
        delta_e_0+D_e,delta_e_0,delta_e_0], t, 'linear');
    
    theta0=gamma_0 - myAC.mu_x + alpha_0;
    y0=[V_0,alpha_0,q_0,0,z_0,theta0]; %7.76a dal Quaderno 7 /vettore di stato_0
    
    options=odeset('AbsTol',10^-9,'RelTol',10^-9);
    [tspan,y]=ode45(@eqLongDynamicStickFixed,[0,t_fin],y0,options); %Risoluzione dell'equazione 
    
    tspan_CasoA=tspan; y_CasoA=y; delta_e_CasoA=delta_e(tspan);
    
    [~, ~, ~, rho] = atmosisa(-y(:,5)); %rho=rho';
    mu_rel = (myAC.W/g)./(rho*myAC.S*myAC.b);
    gamma=y(:,6)+myAC.mu_x-y(:,2);
    fxA_CasoA=-(sin(gamma)+ ((delta_T_0*myAC.T/myAC.W).*cos(y(:,2) - myAC.mu_x + myAC.mu_T) ...
        -sin(gamma) -((rho.*y(:,1).^2)/(2*(myAC.W/myAC.S))) ...
        .*(myAC.CD_0 + myAC.K.*((myAC.CL_alpha.*y(:,2) ...
        + myAC.CL_delta_e.*delta_e_CasoA + myAC.CL_delta_s.*delta_s_0).^myAC.m)))); %eq (7.70)
    F2_CasoA = (1./(1+((myAC.mac/myAC.b).*myAC.CL_alpha_dot)./(4.*mu_rel))).*( 1. - myAC.CL_q.*(myAC.mac/myAC.b)./(4.*mu_rel) ).*y(:,3) ...
        -(g./y(:,1)).*(delta_T_0*myAC.T/myAC.W).*sin(y(:,2) - myAC.mu_x + myAC.mu_T) ...
        +(g./y(:,1)).*cos(gamma) -((rho.*y(:,1).*g)./(2.*(myAC.W/myAC.S))) ...
        .*( myAC.CL_alpha.*y(:,2) + myAC.CL_delta_e.*delta_e_CasoA ...
        + myAC.CL_delta_s.*delta_s_0);                                          % alpha_dot 7.63
    fzA_CasoA = cos(gamma)+y(:,1)./g.*(y(:,3)-F2_CasoA); % eq (7.71)
    
    %% Risoluzione delle equazioni del moto e calcolo fattori di carico (CASO B)
    
    t1=1; t2=1.5; t3=2; t_fin=10;
    D_e=convang(-3.,'deg','rad');
    
    delta_e = @(t) ...
        interp1([0,t1,t2,t3,t_fin],[delta_e_0,delta_e_0,...
        delta_e_0+D_e,delta_e_0,delta_e_0], t, 'linear');
    
    theta0=gamma_0-myAC.mu_x+alpha_0;
    y0=[V_0,alpha_0,q_0,0,z_0,theta0];
    
    options=odeset('AbsTol',10^-9,'RelTol',10^-9);
    [tspan,y]=ode45(@eqLongDynamicStickFixed,[0,t_fin],y0,options);
    
    tspan_CasoB=tspan; y_CasoB=y; delta_e_CasoB=delta_e(tspan);
    
    [~, ~, ~, rho] = atmosisa(-y(:,5)); %rho=rho';
    mu_rel = (myAC.W/g)./(rho*myAC.S*myAC.b);
    gamma=y(:,6)+myAC.mu_x-y(:,2);
    fxA_CasoB=-(sin(gamma)+ ((delta_T_0*myAC.T/myAC.W).*cos(y(:,2) - myAC.mu_x + myAC.mu_T) ...
        -sin(gamma) -((rho.*y(:,1).^2)/(2*(myAC.W/myAC.S))) ...
        .*(myAC.CD_0 + myAC.K.*((myAC.CL_alpha.*y(:,2) ...
        + myAC.CL_delta_e.*delta_e_CasoB + myAC.CL_delta_s.*delta_s_0).^myAC.m))));
    F2_CasoB = (1./(1+((myAC.mac/myAC.b).*myAC.CL_alpha_dot)./(4.*mu_rel))).*( 1. - myAC.CL_q.*(myAC.mac/myAC.b)./(4.*mu_rel) ).*y(:,3) ...
        -(g./y(:,1)).*(delta_T_0*myAC.T/myAC.W).*sin(y(:,2) - myAC.mu_x + myAC.mu_T) ...
        +(g./y(:,1)).*cos(gamma) -((rho.*y(:,1).*g)./(2.*(myAC.W/myAC.S))) ...
        .*( myAC.CL_alpha.*y(:,2) + myAC.CL_delta_e.*delta_e_CasoB ...
        + myAC.CL_delta_s.*delta_s_0);
    fzA_CasoB = cos(gamma)+y(:,1)./g.*(y(:,3)-F2_CasoB);
    
    
    %% Plot
    
    figure(1);
    plot(tspan_CasoA,convang(delta_e_CasoA,'rad','deg'),'-k',tspan_CasoB,convang(delta_e_CasoB,'rad','deg'),'-b','linewidth',1);
    xlabel('t (s)'); ylabel('\delta_e (°)'); axis([0,10,-4.5,-0.8])
    set(get(gca,'YLabel'),'Rotation',90); legend('Caso A','Caso B');
    grid on; grid minor;
    
    figure(2);
    subplot(3,2,1);
    plot(tspan_CasoA,y_CasoA(:,1)-V_0,'-k',tspan_CasoB,y_CasoB(:,1)-V_0,'-b','linewidth',1);
    xlabel('t (s)'); ylabel('\DeltaV (m/s)');
    set(get(gca,'YLabel'),'Rotation',90); legend('Caso A','Caso B');
    grid on; grid minor;
    
    subplot(3,2,3);
    plot(tspan_CasoA,convang(y_CasoA(:,2)-alpha_0,'rad','deg'),'-k',tspan_CasoB,convang(y_CasoB(:,2)-alpha_0,'rad','deg'),'-b','linewidth',1);
    xlabel('t (s)'); ylabel('\Delta\alpha (°)');
    set(get(gca,'YLabel'),'Rotation',90); legend('Caso A','Caso B');
    grid on; grid minor;
    
    subplot(3,2,5);
    plot(tspan_CasoA,convangvel(y_CasoA(:,3)-q_0,'rad/s','deg/s'),'-k',tspan_CasoB,convangvel(y_CasoB(:,3)-q_0,'rad/s','deg/s'),'-b','linewidth',1);
    xlabel('t (s)'); ylabel('\Deltaq (°/s)');
    set(get(gca,'YLabel'),'Rotation',90); legend('Caso A','Caso B');
    grid on; grid minor;
    
    subplot(3,2,2);
    plot(tspan_CasoA,y_CasoA(:,4),'-k',tspan_CasoB,y_CasoB(:,4),'-b','linewidth',1);
    xlabel('t (s)'); ylabel('\Deltax_{E,G} (m)');
    set(get(gca,'YLabel'),'Rotation',90); legend('Caso A','Caso B');
    grid on; grid minor;
    
    subplot(3,2,4);
    plot(tspan_CasoA,-y_CasoA(:,5)+z_0,'-k',tspan_CasoB,-y_CasoB(:,5)+z_0,'-b','linewidth',1);
    xlabel('t (s)'); ylabel('\Deltah (m)');
    set(get(gca,'YLabel'),'Rotation',90); legend('Caso A','Caso B');
    grid on; grid minor;
    
    subplot(3,2,6);
    plot(tspan_CasoA,convang(y_CasoA(:,6),'rad','deg'),'-k',tspan_CasoA,convang(y_CasoA(:,6)+myAC.mu_x-y_CasoA(:,2),'rad','deg'),'-.k','linewidth',1); hold on;
    plot(tspan_CasoB,convang(y_CasoB(:,6),'rad','deg'),'-b',tspan_CasoB,convang(y_CasoB(:,6)+myAC.mu_x-y_CasoB(:,2),'rad','deg'),'-.b','linewidth',1);
    
    xlabel('t (s)'); ylabel('\theta (°)');
    set(get(gca,'YLabel'),'Rotation',90);
    legend('\theta (deg), A','\gamma (deg), A','\theta (deg), B','\gamma (deg), B');
    grid on; grid minor;
    
    figure(3)
    plot(tspan_CasoA,fxA_CasoA,'-k',tspan_CasoB,fxA_CasoB,'-b','linewidth',1); hold on;
    plot(tspan_CasoA,fzA_CasoA,'--k',tspan_CasoB,fzA_CasoB,'--b','linewidth',1); hold on;
    
    xlabel('t (s)'); %axis([0,10,-4.5,-0.8])
    set(get(gca,'YLabel'),'Rotation',90); legend('f_{xA}, Caso A','f_{xA}, Caso B','f_{zA}, Caso A','f_{zA}, Caso B');
    grid on; grid minor;
    
end






