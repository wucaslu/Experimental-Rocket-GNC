function summary = summarizeRocketAerodynamics(out,Rocket,baseline,outputDirectory)
%SUMMARIZEROCKETAERODYNAMICS Report component aerodynamics and flight changes.
% Pure analysis of logged 17-state plant data; configuration is not changed.
% Angles use the local airflow at each component CP, including body rotation.
% Flight coefficient/angle diagnostics are restricted to phase 2 (free flight)
% before recovery inflation. The powered subset separates boost behavior from
% the large angles possible near apogee. Geometry is a demonstration example.
% References for the component normal-force, CP and compressibility model:
% https://docs.rocketpy.org/en/latest/_modules/rocketpy/rocket/aero_surface/aero_surface.html
% https://docs.rocketpy.org/en/latest/_modules/rocketpy/rocket/aero_surface/fins/fins.html
% https://docs.rocketpy.org/en/latest/_modules/rocketpy/rocket/aero_surface/nose_cone.html
% https://docs.rocketpy.org/en/latest/_modules/rocketpy/rocket/aero_surface/tail.html

    assert(isstruct(baseline) && all(isfield(baseline,{'out','Rocket'})), ...
        'RocketPlant:MissingBaseline','Baseline must contain out and Rocket.');
    if ~isfolder(outputDirectory), mkdir(outputDirectory); end
    [t,x] = loggedTruth(out);
    [oldT,oldX] = loggedTruth(baseline.out);
    assert(size(Rocket.AeroSurfaceCP,1)==3, ...
        'RocketPlant:UnexpectedSurfaces','This report expects nose, fin set and tail.');
    surfaceNames = {'Nose','Fin set','Tail'};

    freeIndices = find(x(:,14)==2);
    assert(~isempty(freeIndices),'RocketPlant:NoFreeFlight', ...
        'Simulation must contain phase 2 free flight.');
    angles = zeros(numel(freeIndices),3);
    margin = zeros(numel(freeIndices),1);
    cmMach = zeros(numel(freeIndices),1);
    cmPressure = zeros(numel(freeIndices),1);
    for k=1:numel(freeIndices)
        i=freeIndices(k);
        [~,~,aero]=rocketPlantAerodynamics(x(i,:).',t(i),Rocket);
        angles(k,:)=rad2deg(aero.alpha).';
        margin(k)=aero.staticMargin;
        cmMach(k)=aero.machCM;
        cmPressure(k)=aero.dynamicPressureCM;
    end
    assert(all(isfinite([angles(:);margin;cmMach;cmPressure])), ...
        'RocketPlant:InvalidAeroReport','Aerodynamic diagnostics must be finite.');
    freeT=t(freeIndices);
    boostMask=freeT>=Rocket.IgnitionTime & ...
        freeT<Rocket.IgnitionTime+Rocket.ThrustTime(end);
    assert(any(boostMask),'RocketPlant:NoPoweredFreeFlight', ...
        'The report requires powered free flight after leaving the rail.');
    [freeMaxAlpha,freeRow,freeSurface]=largestAngle(angles);
    [boostMaxAlpha,boostRow,boostSurface]=largestAngle(angles(boostMask,:));
    boostRows=find(boostMask);
    boostRow=boostRows(boostRow);
    [minMargin,marginRow]=min(margin);

    % Zero wind and zero body rates isolate geometry/Mach dependence. The
    % atmosphere is sampled at the launch-site MSL height for reproducibility.
    sweepRocket=Rocket;
    sweepRocket.WindNED=zeros(size(Rocket.WindNED));
    [~,~,soundSpeed]=rocketPlantAtmosphere(Rocket.AltitudeMSL);
    sweepMach=unique([linspace(0,2,401),0.8,1.1]).';
    slopes=zeros(numel(sweepMach),3);
    weightedCP=zeros(numel(sweepMach),1);
    sweepMargin=zeros(numel(sweepMach),1);
    for k=1:numel(sweepMach)
        aero=coefficientAtMach(sweepMach(k),soundSpeed,sweepRocket);
        slopes(k,:)=aero.cna.';
        weightedCP(k)=aero.cpWeighted(3);
        sweepMargin(k)=aero.staticMargin;
    end

    coefficientMach=[0.3;1];
    coefficientSamples=repmat(struct('Mach',0,'ComponentCNaPerRad',zeros(3,1), ...
        'TotalCNaPerRad',0,'WeightedCPBodyZM',0,'StaticMarginDiameters',0),2,1);
    for k=1:numel(coefficientMach)
        aero=coefficientAtMach(coefficientMach(k),soundSpeed,sweepRocket);
        coefficientSamples(k)=struct('Mach',coefficientMach(k), ...
            'ComponentCNaPerRad',aero.cna,'TotalCNaPerRad',sum(aero.cna), ...
            'WeightedCPBodyZM',aero.cpWeighted(3), ...
            'StaticMarginDiameters',aero.staticMargin);
    end
    components=repmat(struct('Name','','CPBodyM',zeros(3,1)),3,1);
    for k=1:3
        components(k).Name=surfaceNames{k};
        components(k).CPBodyM=Rocket.AeroSurfaceCP(k,:).';
    end
    newMetrics=flightMetrics(t,x,Rocket);
    oldMetrics=flightMetrics(oldT,oldX,baseline.Rocket);
    estimated=[out.North.signals.values(:),out.East.signals.values(:), ...
        out.Up.signals.values(:)];
    truth=[out.NorthTrue.signals.values(:),out.EastTrue.signals.values(:), ...
        out.UpTrue.signals.values(:)];
    assert(isequal(size(estimated),size(truth)) && size(truth,1)==numel(t) && ...
        all(isfinite(estimated(:))) && all(isfinite(truth(:))), ...
        'RocketPlant:InvalidNavigationReport','Navigation logging must be finite and aligned.');
    newMetrics.FinalPositionErrorM=norm(estimated(end,:)-truth(end,:));

    summary=struct('DemonstrationGeometry',true, ...
        'PositionConvention','Body +Z noseward, component CP relative to fixed CM', ...
        'ComponentOrder',{surfaceNames},'Components',components, ...
        'CoefficientSamples',coefficientSamples, ...
        'CoefficientSweepMachRange',[sweepMach(1);sweepMach(end)], ...
        'CoefficientSweepAltitudeMSLM',Rocket.AltitudeMSL, ...
        'MinimumFreeFlightStaticMarginDiameters',minMargin, ...
        'MinimumMarginTimeSinceIgnitionS',freeT(marginRow)-Rocket.IgnitionTime, ...
        'MaximumFreeFlightAlphaDeg',freeMaxAlpha, ...
        'MaximumFreeFlightAlphaSurface',surfaceNames{freeSurface}, ...
        'MaximumFreeFlightAlphaTimeSinceIgnitionS',freeT(freeRow)-Rocket.IgnitionTime, ...
        'MachAtMaximumFreeFlightAlpha',cmMach(freeRow), ...
        'DynamicPressureAtMaximumFreeFlightAlphaPa',cmPressure(freeRow), ...
        'MaximumPoweredFreeFlightAlphaDeg',boostMaxAlpha, ...
        'MaximumPoweredFreeFlightAlphaSurface',surfaceNames{boostSurface}, ...
        'MaximumPoweredFreeFlightAlphaTimeSinceIgnitionS',freeT(boostRow)-Rocket.IgnitionTime, ...
        'AngleDomainNote','Linear Barrowman lift is extrapolated at large angle of attack', ...
        'Baseline',oldMetrics,'ComponentAerodynamics',newMetrics, ...
        'ApogeeHeightChangeM',newMetrics.ApogeeHeightAGLM-oldMetrics.ApogeeHeightAGLM, ...
        'PeakSpeedChangeMps',newMetrics.PeakSpeedMps-oldMetrics.PeakSpeedMps, ...
        'LandingTimeChangeS',newMetrics.LandingTimeSinceIgnitionS-oldMetrics.LandingTimeSinceIgnitionS);
    fid=fopen(fullfile(outputDirectory,'rocket-aerodynamics-summary.json'),'w');
    assert(fid>=0,'RocketPlant:ReportWriteFailure','Cannot write aerodynamic report.');
    cleanup=onCleanup(@() fclose(fid));
    fprintf(fid,'%s\n',jsonencode(summary,'PrettyPrint',true));
    coefficientTable=array2table([sweepMach,slopes,sum(slopes,2),weightedCP,sweepMargin], ...
        'VariableNames',{'Mach','NoseCNa_perRad','FinSetCNa_perRad','TailCNa_perRad', ...
        'TotalCNa_perRad','WeightedCPBodyZ_m','StaticMargin_diameters'});
    writetable(coefficientTable,fullfile(outputDirectory,'rocket-aerodynamic-coefficients.csv'));

    f=figure('Color','white','Name','Component aerodynamics comparison', ...
        'Visible','off','Position',[100 100 1100 760]);
    figureCleanup=onCleanup(@() close(f));
    layout=tiledlayout(f,2,2,'TileSpacing','compact','Padding','compact');
    nexttile(layout);
    plot(sweepMach,[slopes,sum(slopes,2)],'LineWidth',1.5);
    grid on; xlabel('Mach'); ylabel('Normal-force slope (rad^{-1})');
    legend('Nose','Fin set','Tail','Total','Location','best');
    title('Component normal-force slopes');
    nexttile(layout);
    yyaxis left; plot(sweepMach,weightedCP,'LineWidth',1.5);
    ylabel('Weighted CP body Z (m)');
    yyaxis right; plot(sweepMach,sweepMargin,'LineWidth',1.5);
    ylabel('Static margin (body diameters)');
    grid on; xlabel('Mach'); title('CP relative to the fixed CM');
    legend('CP','Static margin','Location','best');
    oldWindow=oldT>=baseline.Rocket.IgnitionTime-2 & ...
        oldT<=oldMetrics.LandingTimeS+2;
    newWindow=t>=Rocket.IgnitionTime-2 & t<=newMetrics.LandingTimeS+2;
    nexttile(layout);
    plot(oldT(oldWindow)-baseline.Rocket.IgnitionTime,-oldX(oldWindow,3), ...
        t(newWindow)-Rocket.IgnitionTime,-x(newWindow,3),'LineWidth',1.4);
    grid on; xlabel('Time since ignition (s)'); ylabel('Height AGL (m)');
    legend('Combined CP baseline','Component model','Location','best');
    title('Trajectory response');
    nexttile(layout);
    plot(oldT(oldWindow)-baseline.Rocket.IgnitionTime,vecnorm(oldX(oldWindow,4:6),2,2), ...
        t(newWindow)-Rocket.IgnitionTime,vecnorm(x(newWindow,4:6),2,2),'LineWidth',1.4);
    grid on; xlabel('Time since ignition (s)'); ylabel('Ground speed (m/s)');
    legend('Combined CP baseline','Component model','Location','best');
    title('Speed response');
    title(layout,'Demonstration geometry: nose, fin set and tail at individual CPs');
    exportgraphics(f,fullfile(outputDirectory,'rocket-aerodynamics-comparison.png'),'Resolution',170);
    exportgraphics(f,fullfile(outputDirectory,'rocket-aerodynamics-comparison.pdf'),'ContentType','vector');
end

function [t,x]=loggedTruth(out)
    t=out.RocketPlantState.Time(:);
    x=out.RocketPlantState.Data;
    assert(size(x,2)==17 && size(x,1)==numel(t) && all(isfinite(x(:))) && ...
        all(isfinite(t)) && all(diff(t)>0), ...
        'RocketPlant:InvalidTruth','Expected finite, increasing 17-state truth logging.');
end

function aero=coefficientAtMach(mach,soundSpeed,Rocket)
    state=zeros(17,1);
    state(10)=1; % identity body-to-NED quaternion; airflow is along body +Z
    state(6)=mach*soundSpeed;
    state(14)=2;
    [~,~,aero]=rocketPlantAerodynamics(state,Rocket.IgnitionTime+0.5,Rocket);
end

function [value,row,column]=largestAngle(angles)
    [value,index]=max(angles(:));
    [row,column]=ind2sub(size(angles),index);
end

function metrics=flightMetrics(t,x,Rocket)
    [height,apogeeIndex]=max(-x(:,3));
    [speed,speedIndex]=max(vecnorm(x(:,4:6),2,2));
    landed=find(x(:,14)==5,1);
    assert(~isempty(landed),'RocketPlant:NoLanding','Both runs must contain landing.');
    metrics=struct('ApogeeHeightAGLM',height,'ApogeeTimeS',t(apogeeIndex), ...
        'ApogeeTimeSinceIgnitionS',t(apogeeIndex)-Rocket.IgnitionTime, ...
        'PeakSpeedMps',speed,'PeakSpeedTimeS',t(speedIndex), ...
        'PeakBodyRateRadps',max(vecnorm(x(:,11:13),2,2)), ...
        'LandingTimeS',t(landed), ...
        'LandingTimeSinceIgnitionS',t(landed)-Rocket.IgnitionTime);
end
