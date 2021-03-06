function [fig] = myplotaxes(fig, Origin, vExtent,varargin)

if isempty(varargin)
T=eye(3);
else
vEulerAngles=varargin{1};
psi   = vEulerAngles(1);
theta = vEulerAngles(2);
phi   = vEulerAngles(3);
T = angle2dcm(psi, theta, phi, 'ZYX')';
end

Lversx= T(:,1)*vExtent(1);
Lversy= T(:,2)*vExtent(2);
Lversz= T(:,3)*vExtent(3);

%% Plot axes
quiver3( ...
    Origin(1),Origin(2),Origin(3), ...
    Lversx(1),Lversx(2),Lversx(3), ...
    'AutoScale', 'off', 'color',[1 0.3 0.3],'linewidth',2.5 ...
,'linestyle','-'...
);
quiver3( ...
    Origin(1),Origin(2),Origin(3), ...
    Lversy(1),Lversy(2),Lversy(3), ...
    'AutoScale', 'off', 'color',[0.3 1 0.3],'linewidth',2.5 ...
    ,'linestyle','-'...
);
quiver3( ...
    Origin(1),Origin(2),Origin(3), ...
    Lversz(1),Lversz(2),Lversz(3), ...
    'AutoScale', 'off', 'color',[0.3 0.3 1],'linewidth',2.5 ...
    ,'linestyle','-'...
);

end