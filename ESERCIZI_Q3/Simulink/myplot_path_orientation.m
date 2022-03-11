function [ fig ] = myplot_path_orientation(fig,shape,mXYZe,mEA,options)
%% Sanity checks
if ( isempty(shape) || isempty(mXYZe) || isempty(mEA) )
    disp('  plotTrajectoryAndBodyE - Error:');
    disp('      Empty matrices.');
    return
end
if ( size(mXYZe,1)~=size(mEA,1) )
    disp('  plotTrajectoryAndBodyE - Error:');
    disp('      Size mismatch between Euler angle and CG coordinate arrays.');
    return
end
if ( (size(mXYZe,2) ~=3) || (size(mEA,2) ~= 3) )
    disp('  plotTrajectoryAndBodyE - Error:');
    disp('      Need 3 CoG coordinates and 3 Euler angles.');
    return
end

if ~isfield(options,'samples')
    disp('  plotTrajectoryAndBodyE - Error:');
    disp('      Check options.samples');
    return
end
% if ~isfield(options,'theView')
%     disp('  plotTrajectoryAndBodyE - Error:');
%     disp('      Check options.theView');
%     return
% end
if ~isfield(options,'bodyAxes')
    disp('  plotTrajectoryAndBodyE - Error:');
    disp('      Check options.bodyAxes');
    return
end
if ~isfield(options,'helperLines')
    disp('  plotTrajectoryAndBodyE - Error:');
    disp('      Check options.helperLines');
    return
end
if ~isfield(options,'trajectory')
    disp('  plotTrajectoryAndBodyE - Error:');
    disp('      Check options.trajectory');
    return
end

if ~isnumeric(options.samples)
    disp('  plotTrajectoryAndBodyE - Error:');
    disp('      Check options.samples, must be a numeic vector.');
    return
end
if ~isvector(options.samples)
    disp('  plotTrajectoryAndBodyE - Error:');
    disp('      Check options.samples, must be a numeic vector.');
    return
else
    if ~isempty(find(options.samples > size(mXYZe,1)))
        disp('  plotTrajectoryAndBodyE - Error:');
        disp('      Some indices in options.samples out of range. Check.');
        return
    end
end

if ( ~isfield(options.helperLines,'show') )
    options.helperLines.show = false;
end

if ( ~isfield(options.trajectory,'show') )
    options.trajectory.show = false;
end

%------ end of sanity checks


%% samples plot
hold on
for isam = 1:length(options.samples)
    i = options.samples(isam);
    myplotbody(fig,shape,mXYZe(i,:),mEA(i,:));
    myplotaxes(fig,mXYZe(i,:),options.bodyAxes.Laxes,mEA(i,:));
if (options.helperLines.show)
        myplothelperlines(fig, mXYZe(i,:), options.helperLines);
end
end 

%% Plot the flight path and trajectory
if (options.trajectory.show)
    myplotTrajectory(fig,mXYZe,options.trajectory);
end

end