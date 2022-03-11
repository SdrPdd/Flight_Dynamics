%graphic settings
options.shapeScaleFactor = 450.0;
options.theView = [135 15];
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