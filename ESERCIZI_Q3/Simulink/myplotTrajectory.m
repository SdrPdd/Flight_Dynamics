function [fig] = myplotTrajectory(fig,mXYZe,varargin)
if isempty(varargin)
options.lineStyle='-';
options.lineColor='k';
options.lineWidth=1.5;
else
options=varargin{1};
end
% Ground Track
plot3( ...
    mXYZe(:,1), mXYZe(:,2), 0.0.*mXYZe(:,3)./mXYZe(:,3)*max(mXYZe(:,3)), ...
    '-', 'color', [.5 .5 .5])

% Trajectory in a vertical plane
plot3(mXYZe(:,1), mXYZe(:,2)./mXYZe(:,2)*max(mXYZe(:,2)), mXYZe(:,3), '-', 'color', [.5 .5 .5])

% Trajectory in a vertical plane
plot3(mXYZe(:,1)./mXYZe(:,1)*max(mXYZe(:,1)), mXYZe(:,2), mXYZe(:,3), '-', 'color', [.5 .5 .5])

% 3D trajectory
plot3(mXYZe(:,1), mXYZe(:,2), mXYZe(:,3), ...
    'LineStyle', options.lineStyle, ...
    'Color', options.lineColor, ...
    'LineWidth', options.lineWidth);

end