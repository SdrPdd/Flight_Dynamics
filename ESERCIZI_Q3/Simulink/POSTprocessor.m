%% POST PROCESS SIMULINK 
vtime= tspsithetaphi.Time;
mpsithetaphi=tspsithetaphi.Data;
 
mxeyeze=[tsxeyeze.Data(1,:)',tsxeyeze.Data(2,:)',tsxeyeze.Data(3,:)'];
%graphic settings
options.shapeScaleFactor = 40.0;
options.samples = [1,40,80,120,160];
options.theView = [105 15];
%options.theView = [135 15];
% body axes settings
options.bodyAxes.show = true;
options.bodyAxes.Laxes=[1.5*options.shapeScaleFactor,...
    2*options.shapeScaleFactor,2*options.shapeScaleFactor];
options.bodyAxes.lineWidth = 2.5;
% helper lines
options.helperLines.show = false;
options.helperLines.lineStyle = ':';
options.helperLines.lineColor = 'k';
options.helperLines.lineWidth = 1.5;
% trajectory
options.trajectory.show = true;
options.trajectory.lineStyle = '-';
options.trajectory.lineColor = 'k';
options.trajectory.lineWidth = 0.5;
shape = loadAircraftMAT('aircraft_pa24.mat',options.shapeScaleFactor);

%% Setup the figure/scene for 3D visualization
options.samples = 1:20:length(vtime);

fig1 = figure(1);
AX1=axes('parent',fig1);
grid on;
hold on;
light('Position',[1 0 -4],'Style','local');
% Trick to have Ze pointing downward and correct visualization
set(AX1,'XDir','reverse','ZDir','reverse');
xlabel(AX1,'x_E (m)'); ylabel(AX1,'y_E (m)'); zlabel(AX1,'z_E (m)')
view(options.theView)
%% Plot Earth axes
xleng_e = max([max(abs(tsxeyeze.Data(1,:))),5]);
yleng_e = max([max(abs(tsxeyeze.Data(2,:))),5]);
zleng_e = 0.2*xleng_e;

Origin = [0,0,ze0];
vleng_e = [xleng_e,yleng_e,zleng_e];
myplotaxes(fig1, Origin, vleng_e);


%% Plot body and trajectory
myplot_path_orientation(fig1, shape,mxeyeze, mpsithetaphi, options);
