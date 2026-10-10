function summary = summarizeRocketPlant(out,Rocket,outputDirectory)
%SUMMARIZEROCKETPLANT Export integrated truth and navigation results.
% Requires plant state/debug logs; navigation position logs are optional.
% Results concern the configured demonstration physics, not a measured fit.
    if ~isfolder(outputDirectory), mkdir(outputDirectory); end
    t = out.RocketPlantState.Time;
    x = out.RocketPlantState.Data;
    dbg = out.RocketPlantDebug.Data;
    assert(size(x,2)==17 && size(dbg,2)==18 && all(isfinite(x(:))) && ...
        all(isfinite(dbg(:))),'RocketPlant:InvalidTruth','Truth must be finite.');
    assert(numel(out.tout)==numel(t) && max(abs(out.tout-t))<1e-9);
    navigationLogs = {'North','East','Up','NorthTrue','EastTrue','UpTrue'};
    hasNavigation = all(ismember(navigationLogs,who(out)));
    if hasNavigation
        estimated = [out.North.signals.values(:),out.East.signals.values(:),out.Up.signals.values(:)];
        truth = [out.NorthTrue.signals.values(:),out.EastTrue.signals.values(:),out.UpTrue.signals.values(:)];
        assert(isequal(size(estimated),size(truth)) && size(truth,1)==numel(t));
        assert(all(isfinite(estimated(:))),'RocketPlant:InvalidEstimate','Navigation must be finite.');
        error = vecnorm(estimated-truth,2,2);
        inFlight = x(:,14)>0 & x(:,14)<5;
        launch = t>=Rocket.IgnitionTime & t<=Rocket.IgnitionTime+15;
    end
    [apogeeHeight,apogeeIndex] = max(-x(:,3));
    [peakSpeed,peakSpeedIndex] = max(vecnorm(x(:,4:6),2,2));
    landingIndex = find(x(:,14)==5,1);
    assert(~isempty(landingIndex),'RocketPlant:NoLanding','Simulation must include landing.');
    summary = struct('DemonstrationParameters',true,'StopTimeS',t(end), ...
        'ApogeeHeightAGLM',apogeeHeight,'ApogeeSampleTimeS',t(apogeeIndex), ...
        'ApogeeEventTimeS',x(end,15),'DrogueDeploymentTimeS',x(end,16), ...
        'MainDeploymentTimeS',x(end,17),'LandingTimeS',t(landingIndex), ...
        'PeakSpeedMps',peakSpeed,'PeakSpeedTimeS',t(peakSpeedIndex), ...
        'PeakBodyRateRadps',max(vecnorm(x(:,11:13),2,2)), ...
        'MaximumQuaternionNormError',max(abs(vecnorm(x(:,7:10),2,2)-1)));
    if hasNavigation
        summary.FlightPositionRmsErrorM = sqrt(mean(error(inFlight).^2));
        summary.LaunchPeakPositionErrorM = max(error(launch));
    end
    summary.LandingPositionNEUM = [x(end,1);x(end,2);Rocket.AltitudeMSL];
    summary.MainDeploymentHeightAGLM = interp1(t,-x(:,3),x(end,17));
    summary.DryMassKg = Rocket.DryMass;
    summary.IgnitionMassKg = Rocket.DryMass+Rocket.PropellantMass;
    if ~hasNavigation
        summary.NavigationStatus = 'Missing navigation position logs; metrics omitted.';
    end
    fid = fopen(fullfile(outputDirectory,'integrated-flight-summary.json'),'w');
    assert(fid>=0);
    cleanup = onCleanup(@() fclose(fid));
    fprintf(fid,'%s\n',jsonencode(summary,'PrettyPrint',true));
    trajectory = array2table([t,x,dbg], ...
        'VariableNames',{'Time_s','North_m','East_m','Down_m', ...
        'vNorth_mps','vEast_mps','vDown_mps','qx','qy','qz','qw', ...
        'wx_radps','wy_radps','wz_radps','Phase','ApogeeEvent_s', ...
        'DrogueEvent_s','MainEvent_s','DebugPhase','Mass_kg','Thrust_N', ...
        'GroundSpeed_mps','AirSpeed_mps','Mach','Density_kgpm3', ...
        'Pressure_Pa','DrogueCdS_m2','MainCdS_m2','HeightAGL_m', ...
        'DebugApogee_s','DebugDrogue_s','DebugMain_s','aNorth_mps2', ...
        'aEast_mps2','aDown_mps2','AngularAccelNorm_radps2'});
    if hasNavigation, trajectory.PositionError_m = error; end
    writetable(trajectory,fullfile(outputDirectory,'generated-rocket-trajectory.csv'));

    f = figure('Color','white','Name','Physics-driven rocket checkup');
    tiledlayout(f,3,2,'TileSpacing','compact');
    nexttile;
    if hasNavigation
        plot(t,-x(:,3),t,estimated(:,3)-Rocket.AltitudeMSL);
        legend('Plant','Navigation','Location','best');
    else
        plot(t,-x(:,3)); legend('Plant','Location','best');
    end
    grid on; ylabel('Height AGL (m)');
    nexttile; plot(t,dbg(:,4)); grid on; ylabel('Ground speed (m/s)');
    nexttile; plot(t,dbg(:,3)); grid on; ylabel('Thrust (N)');
    nexttile; plot(t,dbg(:,9:10)); grid on; ylabel('Canopy CdS (m^2)');
    legend('Drogue','Main','Location','best');
    nexttile; plot(t,vecnorm(dbg(:,15:17),2,2)); grid on;
    ylabel('Inertial acceleration (m/s^2)'); xlabel('Simulation time (s)');
    nexttile;
    if hasNavigation
        plot(t,error); grid on; ylabel('Position error (m)'); xlabel('Simulation time (s)');
    else
        axis off;
        text(0.5,0.5,'Navigation position logs unavailable', ...
            'Units','normalized','HorizontalAlignment','center');
    end
    exportgraphics(f,fullfile(outputDirectory,'rocket-flight-checkup.png'),'Resolution',160);
end
