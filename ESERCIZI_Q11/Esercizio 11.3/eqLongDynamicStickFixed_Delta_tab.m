function [dStatedt] = eqLongDynamicStickFixed_Delta_tab(t,state,delta_tab)

global g...
       delta_e...
       delta_s...
       delta_T...
       myAC  

%Assegnazione delle componenti del vettore di stato
V = state(1);
alpha = state(2);
q = state(3);
xEG = state(4);
zEG = state(5);
theta = state(6);
Cm_delta_tab = 0;

T_over_W = myAC.T/myAC.W;
alphaB_plus_muT = alpha - myAC.mu_x + myAC.mu_T;
gamma = theta - alpha + myAC.mu_x;
rhoVsquare_over_TwoWeightSurface = density(-zEG)*V^2/(2*(myAC.W/myAC.S));
CL = myAC.CL_alpha*alpha + myAC.CL_delta_e*delta_e(t)+...
     myAC.CL_delta_s*delta_s(t);
V_dot = g*((delta_T(t)*T_over_W)*cos(alphaB_plus_muT)-...
        sin(gamma) - (rhoVsquare_over_TwoWeightSurface)*...
                     (myAC.CD_0 + myAC.K*CL^myAC.m));
     
mu_rel = (myAC.W/g)/(density(-zEG)*myAC.S*myAC.b);    
macOverb_over_FourMuRel_dot_CLAlphadot = (myAC.mac/myAC.b)*myAC.CL_alpha_dot/(4*mu_rel);
macOverb_over_FourMuRel_dot_CLq = (myAC.mac/myAC.b)*myAC.CL_q/(4*mu_rel);
g_over_V = g/V;
alpha_dot = 1/(1 + macOverb_over_FourMuRel_dot_CLAlphadot)*...
            (q*(1 - macOverb_over_FourMuRel_dot_CLq)-...
             (T_over_W)*delta_T(t)*(g_over_V)*sin(alphaB_plus_muT)+...
             (g_over_V)*cos(gamma)-...
             (rhoVsquare_over_TwoWeightSurface*g_over_V)*CL);
        
V_over_radiusgirationSquare = (V/myAC.k_y)^2;        
macOverb_over_TwoMuRel = (myAC.mac/myAC.b)/(2*mu_rel);
CM_T = (myAC.Cm_T_0 + myAC.Cm_T_alpha*alpha)*delta_T(t);
CM = myAC.Cm_0 + myAC.Cm_alpha*alpha + myAC.Cm_delta_e*delta_e(t)+...
     myAC.Cm_delta_s*delta_s(t) + Cm_delta_tab*delta_tab+...
     (myAC.mac/(2*V))*(myAC.Cm_alpha_dot*alpha_dot + myAC.Cm_q*q);        
q_dot = (V_over_radiusgirationSquare)*(macOverb_over_TwoMuRel)*...
        (CM_T + CM);
    
xEG_dot = V*cos(gamma);

zEG_dot = -V*sin(gamma);

theta_dot = q;
      
dStatedt = [V_dot;alpha_dot;q_dot;xEG_dot;zEG_dot;theta_dot];

end