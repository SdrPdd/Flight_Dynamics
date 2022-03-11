%% FUNZIONI %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function[]=salvafig(N)

saveas(gcf,['Figure',num2str(N),'.',num2str(get(gcf,'Number')),'.emf'],'emf');
end