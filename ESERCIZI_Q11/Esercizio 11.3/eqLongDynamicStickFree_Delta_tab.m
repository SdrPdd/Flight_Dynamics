function [dStatedt] = eqLongDynamicStickFree_Delta_tab(t,x,delta_tab)

global g ...
       delta_s...
       delta_T...
       myAC

V = x(1);
alpha = x(2);
q = x(3);
xEG = x(4);
zEG = x(5);
theta = x(6);
delta_e_dot = x(7);
delta_e = x(8);

rho = density(-zEG);
mu_rel = (myAC.W/g)/(rho*myAC.S*myAC.b);
Cm_delta_tab = 0;

f(1) = g*((delta_T(t)*myAC.T/myAC.W)*...
         cos(alpha - myAC.mu_x + myAC.mu_T)-...
       sin(theta + myAC.mu_x - alpha)-...
       ((rho*V^2)/(2*(myAC.W/myAC.S)))*...
                   (myAC.CD_0 + myAC.K*((myAC.CL_alpha*alpha+...
                                         myAC.CL_delta_e*delta_e+...
                                         myAC.CL_delta_s*delta_s(t))^myAC.m)));

f(2) = 1/(1 + (myAC.mac/myAC.b)/(4*mu_rel)*myAC.CL_alpha_dot)*...
         ((1 - (myAC.mac/myAC.b)/(4*mu_rel)*myAC.CL_q)*q-...
          (delta_T(t)*myAC.T/myAC.W)*(g/V)*...
                      sin(alpha - myAC.mu_x + myAC.mu_T)+...
          (g/V)*cos(theta + myAC.mu_x - alpha)-...
          ((rho*V^2)/(2*(myAC.W/myAC.S)))*(g/V)*...
                     (myAC.CL_alpha*alpha + myAC.CL_delta_e*delta_e+...
                      myAC.CL_delta_s*delta_s(t)));

f(3) = ((V/myAC.k_y)^2*(myAC.mac/myAC.b)/(2*mu_rel))*...
                       ((myAC.Cm_T_0 + myAC.Cm_T_alpha*alpha)+...
                        myAC.Cm_0 + myAC.Cm_alpha*alpha+...
                        myAC.Cm_delta_e*delta_e+...
                        myAC.Cm_delta_s*delta_s(t)+...
                        Cm_delta_tab*delta_tab+...
                        (myAC.mac/(2*V))*myAC.Cm_q*q);

f(4) = V*cos(theta + myAC.mu_x - alpha);

f(5) = -V*sin(theta + myAC.mu_x - alpha);

f(6) = q ;

f(7) = ((myAC.mac_e*myAC.ec_adim)/myAC.k_e^2)*g*cos(theta)+...
       (rho*V^2*myAC.S_e*myAC.mac_e)/(2*myAC.I_e)*...
            (myAC.Ch_e_0 + myAC.Ch_e_alpha*((1 - myAC.DepsDalpha)*...
                                            (alpha - myAC.mu_x) - myAC.eps_0+...
                                            delta_s(t) + myAC.mu_x)+...
             myAC.Ch_e_delta_e*delta_e + myAC.Ch_e_delta_s*delta_s(t)+...
             myAC.Ch_e_delta_tab*delta_tab + (myAC.mac_e/(2*V))*myAC.Ch_e_q*q);

f(8) = delta_e_dot ;

dStatedt = [f(1);f(2);f(3);f(4);f(5);f(6);f(7);f(8)];

end