function [fig] = myplotbody(fig,shape,vXYZe,vEulerAngles)

%% Euler angles
psi   = vEulerAngles(1);
theta = vEulerAngles(2);
phi   = vEulerAngles(3);

%% DCM 
% Transf. matrix from Earth- to body-axes 
Tbe = angle2dcm(psi, theta, phi, 'ZYX')';

%% Vertices in body-axis coordinates
Vb = Tbe*shape.V';
Vb = Vb';

X0 = repmat(vXYZe,size(Vb,1),1);
Vb = Vb + X0;

%% Display aircraft shape
p = patch('faces', shape.F, 'vertices' ,Vb);% parent?????
set(p, 'facecolor', [0 .5 1]);          
set(p, 'EdgeColor','none');
axis equal;
lighting phong


% if isempty(varargin)
% bodyAxesOptions.show=false;
% bodyAxesOptions.magX=1;
% bodyAxesOptions.magY=1;
% bodyAxesOptions.magZ=1;
% bodyAxesOptions.lineWidth=2.5;
% else
%     bodyAxesOptions=varargin{1};
% end
% 
% %% Euler angles
% psi   = vEulerAngles(1);
% theta = vEulerAngles(2);
% phi   = vEulerAngles(3);
% 
% %% DCM 
% % Transf. matrix from Earth- to body-axes 
% Tbe = angle2dcm(psi, theta, phi, 'ZYX')';
% 
% %% Vertices in body-axis coordinates
% Vb = Tbe*shape.V';
% Vb = Vb';
% 
% X0 = repmat(vXYZe,size(Vb,1),1);
% Vb = Vb + X0;
% 
% %% Display aircraft shape
% p = patch('faces', shape.F, 'vertices' ,Vb);
% set(p, 'facecolor', [1 0 0]);          
% set(p, 'EdgeColor','none');
% axis equal;
% lighting phong
% 
% %% plot body-axes
% if (bodyAxesOptions.show)
%     Xb = transpose( ...
%         Tbe * (bodyAxesOptions.magX*[1;0;0]) ...
%         );
%     Yb = transpose( ...
%         Tbe * (bodyAxesOptions.magY*[0;1;0]) ...
%         );
%     Zb = transpose( ...
%         Tbe * (bodyAxesOptions.magZ*[0;0;1]) ...
%         );
%     quiver3( ...
%         vXYZe(1),vXYZe(2),vXYZe(3), ...
%         Xb(1),Xb(2),Xb(3), ...
%         'AutoScale', 'off', 'Color',[1 0 0],'LineWidth',bodyAxesOptions.lineWidth ...
%         );
%     quiver3( ...
%         vXYZe(1),vXYZe(2),vXYZe(3), ...
%         Yb(1),Yb(2),Yb(3), ...
%         'AutoScale', 'off', 'Color',[0 1 0],'LineWidth',bodyAxesOptions.lineWidth ...
%         );
%     quiver3( ...
%         vXYZe(1),vXYZe(2),vXYZe(3), ...
%         Zb(1),Zb(2),Zb(3), ...
%         'AutoScale', 'off', 'Color',[0 0 1],'LineWidth',bodyAxesOptions.lineWidth ...
%         );
% end

end
