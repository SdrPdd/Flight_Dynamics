function [Delta_e] = mismatch(delta_tab,alpha0_rad,theta0_rad,delta_e0_rad)

global t_sim_first...
       t_fin...                 
       xEG_0 zEG_0...
       V0 q0
         
%% Prima fase di volo: volo a comandi bloccati
% Integrazione delle equazioni del moto a 3-DoF
state0_first = [V0 ,alpha0_rad, q0 ,xEG_0 ,zEG_0 ,theta0_rad];

%Integrazione del sistema di equazioni differenziali
options = odeset('RelTol', 1e-9,'AbsTol', 1e-9*ones(1,6));
StickFixed_eq = @(t,state) eqLongDynamicStickFixed_Delta_tab(t,state,delta_tab);
[vTime1,mState1] = ode45(StickFixed_eq,[0 t_sim_first],state0_first,options);

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
options = odeset('Mass',@MassStickFree,'RelTol',1e-9,'AbsTol',1e-9*ones(1,8));
StickFree_eq = @(t,state) eqLongDynamicStickFree_Delta_tab(t,state,delta_tab);
[vTime2,mState2] = ode45(StickFree_eq,[t_sim_first t_fin],state0_second,options);

Delta_e = mState2(end,8) - delta_e0_rad;

end
