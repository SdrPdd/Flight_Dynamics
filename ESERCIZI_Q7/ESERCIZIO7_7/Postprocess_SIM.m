 clear all; close all; clc;
 
 %% POST PROCESS DELL'ESERCIZIO 7.7 
 % Si plottano i grafici di alpha, delta_e e delta_T al variare della
 % velocità
 % Tramite l'utilizzo dello "Steady State Manager" sono state ricavate le
 % condizioni di trim relative a ciascun valore di velocità precedentemente
 % fissati 
 %I risultati (alias report) sono esportati nel workspace. 
 
V=[240,260,280,300,320,340];
alpha_0=[0.0407,0.0352 ,0.0308 ,0.0273, 0.0244,0.022 ]*180/pi;
delta_T0=[0.366,0.417,0.474,0.535,0.603, 0.675];
delta_e0 = [-0.0511,-0.0453,-0.0406,-0.0368,-0.0338,-0.0312]*180/pi;

figure(1)
    subplot 311
    plot(V,alpha_0,'b-o','LineWidth',1.5,'markersize',2.5);
    grid on
    xlim([V(1) V(end)])
    ylim([0 3.5])
    ylabel('$\alpha_{0}(deg)$','interpreter','latex','fontsize',11)
    subplot 312
    plot(V,delta_e0,'b-o','LineWidth',1.5,'markersize',2.5);
    grid on
    xlim([V(1) V(end)])
    ylim([-4 -1])
    ylabel('$\delta_{e,0}(deg)$','interpreter','latex','fontsize',11)
    subplot 313
    plot(V,delta_T0,'b-o','LineWidth',1.5,'markersize',2.5);
    grid on
    xlim([V(1) V(end)])
    xlabel('$V_0 (m/s)$','interpreter','latex','fontsize',11);
    ylim([0 1])
    ylabel('$\delta_{T,0}$','interpreter','latex','fontsize',11)


