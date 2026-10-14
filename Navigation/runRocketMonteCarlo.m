function [results,cases,summary] = runRocketMonteCarlo(numRuns,options)
%RUNROCKETMONTECARLO Reproducible coupled rocket/Navigation uncertainty study.
% [RESULTS,CASES,SUMMARY] = RUNROCKETMONTECARLO(N,OPTIONS) samples rocket/motor,
% aerodynamic drag area, recovery, weather, rail/launch and Navigation initial
% estimates, then runs the OPEN NavigationRocketPlant model through temporary
% SimulationInput overrides. Default N=100. OPTIONS may be a partial override
% of rocketMonteCarloSettings(); uncertainty values are editable examples.
% RESULTS is a scalar-metric table; CASES preserves actual sampled variables
% and draws; SUMMARY includes counts and empirical mean/std/quantiles. Failed
% simulations and landing-censored flights remain separate from completed
% flights. The runner is serial and needs no Parallel Computing Toolbox.
%
% Examples:
%   [results,cases,summary] = runRocketMonteCarlo(100);
%   o=rocketMonteCarloSettings(); o.Seed=123; o.FlightDurationS=40;
%   o.RunSimulations=false; [~,cases]=runRocketMonteCarlo(20,o);
%
% Forecast/CSV data are imported once into a local nominal snapshot. Per-run
% InitFcn is suppressed so it cannot overwrite draws or fetch weather again.
% Onboard calibration and MEKF tuning stay nominal. Sensor noise levels and
% calibration are fixed; existing random streams optionally receive new seeds.
% No persistent variables, set_param, model saving or base-workspace writes.
% Model must be open: its existing PostLoadFcn writes sensor settings to base.
% Outputs: manifest.mat, samples.csv, results.csv, summary.json, optional
% plots and full run MAT files in a unique campaign directory by default.

if nargin<1,numRuns=100;end
validateattributes(numRuns,{'numeric'},{'real','finite','scalar','integer','positive'});
navigationDirectory=fileparts(mfilename('fullpath'));
previousPath=path;pathCleanup=onCleanup(@() path(previousPath));
addpath(navigationDirectory,fullfile(navigationDirectory,'settings'), ...
    fullfile(navigationDirectory,'functions','aux_MonteCarlo'));
defaults=rocketMonteCarloSettings();
if nargin<2,options=struct();end
options=mergeOptions(defaults,options);
validateOptions(options);
model=char(options.Model);
if options.RunSimulations
    assert(bdIsLoaded(model),'RocketMC:OpenModel', ...
        'Open %s before running Monte Carlo. Its PostLoadFcn must not run during this campaign.',model);
    assert(strcmp(get_param(model,'SimulationStatus'),'stopped'), ...
        'RocketMC:Busy','The model must be stopped before starting a campaign.');
end
nominal=prepareNominal(options,navigationDirectory);
if ~isempty(options.ControlEnabled),nominal.Airbrake.Enabled=double(options.ControlEnabled);end
if ~isempty(options.TargetApogeeAGL),nominal.Airbrake.TargetApogeeAGL=options.TargetApogeeAGL;end
folder=char(options.OutputDirectory);
if isempty(folder)
    stamp=char(datetime('now','TimeZone','UTC','Format','yyyyMMdd_HHmmss_SSS'));
    folder=fullfile(navigationDirectory,'RocketPlant','validation','monte-carlo', ...
        sprintf('seed-%d-%s',options.Seed,stamp));
end
assert(~isfile(fullfile(folder,'manifest.mat')),'RocketMC:ExistingCampaign', ...
    'This directory already contains a campaign. Choose a new OutputDirectory.');
if ~isfolder(folder),mkdir(folder);end
options.OutputDirectory=folder;
blank=rocketMonteCarloMetrics([],nominal,1,0,options.TargetToleranceM);
rows=repmat(blank,numRuns,1);
caseTemplate=struct('RunId',0,'Sample',struct(),'Variables',struct(), ...
    'SampleError',"");
cases=repmat(caseTemplate,numRuns,1);
fprintf('Preparing %d Monte Carlo cases, seed %d. Example uncertainty distributions.\n', ...
    numRuns,options.Seed);
for index=1:numRuns
    cases(index).RunId=index;
    rows(index).RunId=index;
    try
        [cases(index).Variables,cases(index).Sample]= ...
            rocketMonteCarloSample(nominal,options,index);
    catch exception
        cases(index).SampleError=string(exception.message);
        rows(index).Status="sample_failed";
        rows(index).Message=cases(index).SampleError;
    end
end
% Save all draws before execution, so interrupted campaigns retain provenance.
save(fullfile(folder,'manifest.mat'),'nominal','options','cases','-v7.3');
sampleRows=flattenSamples(cases);
writetable(sampleRows,fullfile(folder,'samples.csv'));
if options.RunSimulations
    workspaceVariables=whos(get_param(model,'ModelWorkspace'));
    workspaceNames={workspaceVariables.name};
    originalInit=get_param(model,'InitFcn');
    originalStop=get_param(model,'StopTime');
    originalTarget=get_param(Simulink.ID.getFullName([model ':1649']),'Value');
else
    workspaceNames={};
end
for index=1:numRuns
    if strlength(cases(index).SampleError)>0,continue;end
    vars=cases(index).Variables;
    if ~options.RunSimulations
        rows(index).Status="planned";
        rows(index).TargetAGL=vars.Airbrake.TargetApogeeAGL;
        continue
    end
    assert(bdIsLoaded(model),'RocketMC:ModelClosed', ...
        'The model was closed during the campaign. Stop rather than re-run PostLoadFcn.');
    in=Simulink.SimulationInput(model);
    in=in.setModelParameter('InitFcn','','FastRestart','off','LoadExternalInput','off', ...
        'CaptureErrors','on','ReturnWorkspaceOutputs','on','StartTime','0', ...
        'StopTime',num2str(nominal.Rocket.IgnitionTime+options.FlightDurationS,17));
    names=fieldnames(vars);
    for j=1:numel(names)
        if ismember(names{j},workspaceNames)
            in=in.setVariable(names{j},vars.(names{j}),'Workspace',model);
        else
            in=in.setVariable(names{j},vars.(names{j}));
        end
    end
    in=in.setBlockParameter(Simulink.ID.getFullName([model ':1649']), ...
        'Value','Airbrake.TargetApogeeAGL');
    scopeIds=[1015,1016,1017,1006,1007];
    logNames={'MC_Attitude','MC_Velocity','MC_Position','MC_GyroBias','MC_AccelBias'};
    for j=1:numel(scopeIds)
        scopePath=Simulink.ID.getFullName(sprintf('%s:%d',model,scopeIds(j)));
        in=in.setBlockParameter(scopePath,'DataLogging','on');
        in=in.setBlockParameter(scopePath,'DataLoggingVariableName',logNames{j});
        in=in.setBlockParameter(scopePath,'DataLoggingSaveFormat','Dataset');
        in=in.setBlockParameter(scopePath,'DataLoggingLimitDataPoints','off');
    end
    fprintf('Monte Carlo %d/%d ...\n',index,numRuns);
    started=tic;
    try
        out=sim(in);
    catch exception
        rows(index)=rocketMonteCarloMetrics([],vars,index,toc(started),options.TargetToleranceM);
        rows(index).Message=string(exception.identifier)+": "+string(exception.message);
        writetable(struct2table(rows(1:index)),fullfile(folder,'results.csv'));
        continue
    end
    elapsed=toc(started);
    if strlength(string(out.ErrorMessage))>0
        rows(index)=rocketMonteCarloMetrics([],vars,index,elapsed,options.TargetToleranceM);
        rows(index).Message=string(out.ErrorMessage);
    else
        rows(index)=rocketMonteCarloMetrics(out,vars,index,elapsed,options.TargetToleranceM);
    end
    if options.SaveSimulations
        save(fullfile(folder,sprintf('run-%05d.mat',index)),'out','vars','-v7.3');
    end
    fprintf('  %s; peak %.2f m; landed %d; warnings %d\n',rows(index).Status, ...
        rows(index).ApogeeAGL,rows(index).FlightLanded,rows(index).Warnings);
    writetable(struct2table(rows(1:index)),fullfile(folder,'results.csv'));
end
results=struct2table(rows);
summary=summarize(results,options);
if options.RunSimulations
    summary.ModelParametersRestored=strcmp(originalInit,get_param(model,'InitFcn')) && ...
        strcmp(originalStop,get_param(model,'StopTime')) && ...
        strcmp(originalTarget,get_param(Simulink.ID.getFullName([model ':1649']),'Value'));
    assert(summary.ModelParametersRestored,'RocketMC:Restore', ...
        'Temporary model parameters were not restored after simulation.');
else
    summary.ModelParametersRestored=true;
end
save(fullfile(folder,'campaign.mat'),'results','summary','options','cases','nominal','-v7.3');
writetable(results,fullfile(folder,'results.csv'));
writelines(jsonencode(summary,PrettyPrint=true),fullfile(folder,'summary.json'));
if options.MakePlots && options.RunSimulations,makePlots(results,folder);end
fprintf('Monte Carlo saved: %s\n',folder);
end

function options=mergeOptions(defaults,provided)
assert(isstruct(provided) && isscalar(provided),'RocketMC:Options','Options must be a scalar struct.');
options=defaults;
names=fieldnames(provided);
for k=1:numel(names)
    assert(isfield(defaults,names{k}),'RocketMC:OptionName','Unknown option: %s.',names{k});
    if strcmp(names{k},'Uncertainty')
        options.Uncertainty=mergeOptions(defaults.Uncertainty,provided.Uncertainty);
    else
        options.(names{k})=provided.(names{k});
    end
end
end

function validateOptions(options)
validateattributes(options.Seed,{'numeric'},{'scalar','integer','nonnegative','<=',2^32-1});
validateattributes(options.FlightDurationS,{'numeric'},{'scalar','real','finite','positive'});
validateattributes(options.TargetToleranceM,{'numeric'},{'scalar','real','finite','nonnegative'});
validateattributes(options.TruncationSigma,{'numeric'},{'scalar','real','finite','positive','<=',8});
logicalNames={'RunSimulations','SaveSimulations','MakePlots','RandomizeNoiseSeeds'};
for k=1:numel(logicalNames)
    validateattributes(options.(logicalNames{k}),{'logical'},{'scalar'});
end
textNames={'Model','ParameterSet','Scenario','WeatherSource','CdLookupSource','OutputDirectory','MotorDataset'};
for k=1:numel(textNames)
    v=options.(textNames{k});assert((ischar(v) && (isrow(v)||isempty(v))) || ...
        (isstring(v)&&isscalar(v)),'RocketMC:Text','%s must be one string.',textNames{k});
end
assert(isstruct(options.MotorImportOptions) && isscalar(options.MotorImportOptions), ...
    'RocketMC:MotorImportOptions','MotorImportOptions must be a scalar structure.');
if ~isempty(options.ControlEnabled),validateattributes(options.ControlEnabled, ...
        {'numeric','logical'},{'scalar','binary'});end
if ~isempty(options.TargetApogeeAGL),validateattributes(options.TargetApogeeAGL, ...
        {'numeric'},{'scalar','real','finite','positive'});end
names=fieldnames(options.Uncertainty); defaults=rocketMonteCarloSettings();
for k=1:numel(names)
    value=options.Uncertainty.(names{k});
    validateattributes(value,{'numeric'},{'real','finite','nonnegative'});
    assert(isequal(size(value),size(defaults.Uncertainty.(names{k}))), ...
        'RocketMC:UncertaintyShape','Wrong uncertainty size: %s.',names{k});
end
end

function vars=prepareNominal(options,navigationDirectory)
required={'Rocket','RocketInitialState','Weather','Airbrake','AirbrakeInitialState', ...
    'CdLookup','Sensor','MEKF','Environment','GNSSConfig','GNSSInitialState', ...
    'GNSSReceiverStateBus','LLA0','stationaryTime','marginTime','P0','q0','p0', ...
    'v0','ba0','bg0','pressure0','g0'};
if ~isempty(options.NominalVariables)
    vars=options.NominalVariables;
    assert(isstruct(vars)&&isscalar(vars)&&all(isfield(vars,required)), ...
        'RocketMC:Nominal','NominalVariables must contain the complete initialized snapshot.');
    return
end
RocketParameterSet=char(options.ParameterSet); %#ok<NASGU> consumed by initialization script
RocketScenario=char(options.Scenario); %#ok<NASGU> consumed by initialization script
RocketWeatherSource=char(options.WeatherSource); %#ok<NASGU> consumed by initialization script
RocketCdLookupSource=char(options.CdLookupSource); %#ok<NASGU> consumed by initialization script
if ~isempty(options.MotorStaticFireData)
    RocketMotorStaticFireData=options.MotorStaticFireData; %#ok<NASGU> script input
end
if ~isempty(options.MotorDataset)
    RocketMotorDataset=options.MotorDataset; %#ok<NASGU> script input
end
if ~isempty(fieldnames(options.MotorImportOptions))
    RocketMotorImportOptions=options.MotorImportOptions; %#ok<NASGU> script input
end
if ~isempty(options.MotorType)
    RocketMotorType=options.MotorType; %#ok<NASGU> script input
end
if ~isempty(options.MotorProperties)
    RocketMotorProperties=options.MotorProperties; %#ok<NASGU> script input
end
% run executes in THIS function workspace, leaving user's base values alone.
run(fullfile(navigationDirectory,'rocketPlantMain.m'));
vars=struct();
for k=1:numel(required),vars.(required{k})=eval(required{k});end
end

function samples=flattenSamples(cases)
% Scalar/vector draws are inspectable in CSV; complete structures stay in MAT.
valid=find(arrayfun(@(c) strlength(c.SampleError)==0,cases));
if isempty(valid),samples=table();return;end
flat=struct([]);
for index=1:numel(valid)
    value=cases(valid(index)).Sample;
    value=rmfield(value,{'GustPhases','NoiseSeeds','Motor'});
    value.MotorImpulseScale=cases(valid(index)).Sample.Motor.ImpulseScale;
    value.MotorThrustScale=cases(valid(index)).Sample.Motor.ThrustScale;
    value.ConsumedMassKg=cases(valid(index)).Sample.Motor.ConsumedMassKg;
    value.EffectiveExhaustVelocityMps=cases(valid(index)).Sample.Motor.EffectiveExhaustVelocityMps;
    value.BodyReferenceShiftM=cases(valid(index)).Sample.Motor.BodyReferenceShiftM;
    names=fieldnames(value);
    for k=1:numel(names)
        fieldValue=value.(names{k});
        if isnumeric(fieldValue) && isvector(fieldValue)
            value.(names{k})=reshape(fieldValue,1,[]);
        end
    end
    if isempty(flat),flat=value;else,flat(index)=value;end
end
samples=struct2table(flat,'AsArray',true);
end

function summary=summarize(results,options)
summary.Seed=options.Seed;
summary.Runs=height(results);
summary.RunSimulations=options.RunSimulations;
summary.SampleFailures=nnz(results.Status=="sample_failed");
summary.SimulationFailures=nnz(results.Status=="failed");
summary.PlannedRuns=nnz(results.Status=="planned");
summary.CompletedFlights=nnz(results.FlightLanded);
summary.LandingCensored=nnz(results.Status=="censored");
summary.ObservedApogees=nnz(results.ApogeeObserved);
summary.TargetToleranceM=options.TargetToleranceM;
summary.TargetWithinToleranceCount=nnz(results.TargetWithinTolerance);
summary.TargetWithinToleranceFraction=NaN;
if summary.ObservedApogees>0
    summary.TargetWithinToleranceFraction=summary.TargetWithinToleranceCount/summary.ObservedApogees;
end
metrics={'ApogeeAGL','ApogeeErrorM','LandingRangeM','LandingTimeS','MaxMach', ...
    'MaxPositionErrorM','PositionRMSEM','MaxAttitudeErrorDeg','AttitudeRMSEDeg'};
for k=1:numel(metrics)
    values=results.(metrics{k});mask=isfinite(values);
    if ismember(metrics{k},{'ApogeeAGL','ApogeeErrorM'}),mask=mask & results.ApogeeObserved;end
    values=values(mask);
    stats=struct('Count',numel(values),'Mean',NaN,'Std',NaN,'Minimum',NaN, ...
        'P05',NaN,'Median',NaN,'P95',NaN,'Maximum',NaN);
    if ~isempty(values)
        sorted=sort(values);stats.Mean=mean(values);stats.Std=std(values);
        stats.Minimum=sorted(1);stats.Maximum=sorted(end);
        levels=1+(numel(sorted)-1)*[.05,.5,.95];
        lowerValues=reshape(sorted(floor(levels)),1,[]);
        upperValues=reshape(sorted(ceil(levels)),1,[]);
        quantiles=lowerValues.*(ceil(levels)-levels)+upperValues.*(levels-floor(levels));
        exact=floor(levels)==ceil(levels);quantiles(exact)=sorted(levels(exact));
        stats.P05=quantiles(1);stats.Median=quantiles(2);stats.P95=quantiles(3);
    end
    summary.Statistics.(metrics{k})=stats;
end
summary.UncertaintySource='Editable illustrative distributions; not identified operational probabilities.';
summary.OnboardCalibrationFixed=true;
summary.SensorCalibrationFixed=true;
summary.NoiseSeedsRandomized=options.RandomizeNoiseSeeds;
summary.OutputDirectory=options.OutputDirectory;
summary.GustModel='Existing finite Fourier gust ensemble; not validated atmospheric turbulence.';
summary.DragModel='AxialDragScale changes true axial Cd; geometric reference area and onboard calibration preserved.';
end

function makePlots(results,folder)
figureHandle=figure('Name','Rocket Monte Carlo','Color','w','Visible','off');
figureCleanup=onCleanup(@() close(figureHandle));
tiledlayout(2,2);
nexttile;histogram(results.ApogeeAGL(results.ApogeeObserved & isfinite(results.ApogeeAGL)));
xlabel('Apogee AGL [m]');ylabel('Runs');title('Observed flight peaks');grid on;
nexttile;histogram(results.ApogeeErrorM(results.ApogeeObserved));
xlabel('Apogee minus target [m]');ylabel('Runs');title('Observed apogee errors');grid on;
nexttile;scatter(results.LandingEastM,results.LandingNorthM,25,'filled');
xlabel('East [m]');ylabel('North [m]');title('Completed-flight landing positions');axis equal;grid on;
nexttile;scatter(results.MaxPositionErrorM,results.MaxAttitudeErrorDeg,25,'filled');
xlabel('Maximum position error [m]');ylabel('Maximum attitude error [deg]');
title('Navigation errors after ignition');grid on;
exportgraphics(figureHandle,fullfile(folder,'monte-carlo.png'),'Resolution',160);
end
