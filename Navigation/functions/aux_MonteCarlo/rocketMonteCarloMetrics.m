function row=rocketMonteCarloMetrics(out,vars,runId,elapsed,targetTolerance)
%ROCKETMONTECARLOMETRICS Reduce one coupled flight to a stable scalar row.
% Inputs: out is a valid SimulationOutput after the caller has checked its
% ErrorMessage. vars contains that run's Rocket, Airbrake and LLA0 snapshot;
% LLA0(3) is the launch MSL origin [m]. runId is the integer run identifier,
% elapsed is host wall time [s], and targetTolerance is an AGL tolerance [m].
% Required logs: RocketPlantState, AirbrakeControl, AirbrakeLQRDiagnostics,
% WeatherTruth and MC_Attitude/Velocity/Position/GyroBias/AccelBias Datasets.
% q/p/v Dataset elements are truth then Navigation; bias elements are
% Navigation then truth. Quaternions are scalar-last body-to-NED. Root
% position is [North, East, absolute MSL Up]; velocity is NEU.
% Output: row has fixed scalar fields suitable for struct2table. Distances
% are metres, speeds m/s, body rates rad/s, attitude errors degrees, and
% actuator position dimensionless. Event times are ABSOLUTE MODEL seconds,
% consistent with the plant event states, rather than time since ignition.
% Position/attitude errors use ignition through first landing; censored runs
% use their recorded end time. Mach uses physical
% air-relative speed and sound speed, without clipping to the Cd grid.
% Passing out=[] returns a blank failed row, with NaN numeric metrics and
% false event/success flags, for the caller's failure-handling path.
% Missing/malformed/nonfinite logs raise named errors and fail fast rather
% than being silently classified as completed flights. No model, settings,
% workspace or random stream is changed.
if nargin<1,out=[];end
if nargin<2,vars=struct();end
if nargin<3,runId=NaN;end
if nargin<4,elapsed=NaN;end
if nargin<5,targetTolerance=NaN;end
row=blankRow(runId,elapsed);
if isempty(out),return;end
assert(isa(out,'Simulink.SimulationOutput'), ...
    'RocketMonteCarlo:Output','A valid SimulationOutput is required.');
assert(isstruct(vars) && isscalar(vars) && isfield(vars,'Rocket') && ...
    isfield(vars,'Airbrake') && isfield(vars,'LLA0') && ...
    isequal(size(vars.LLA0),[3,1]) && all(isfinite(vars.LLA0)), ...
    'RocketMonteCarlo:Variables','Run snapshot needs Rocket, Airbrake and finite 3-by-1 LLA0.');
assert(isnumeric(targetTolerance) && isscalar(targetTolerance) && ...
    isreal(targetTolerance) && isfinite(targetTolerance) && targetTolerance>=0, ...
    'RocketMonteCarlo:Tolerance','Target tolerance must be finite and nonnegative.');

% Access SimulationOutput directly; simulation errors are handled upstream.
time=double(out.RocketPlantState.Time(:));
assert(~isempty(time) && all(isfinite(time)) && all(diff(time)>0), ...
    'RocketMonteCarlo:Time','Truth timestamps must be finite and strictly increasing.');
truth=alignedSamples(out.RocketPlantState,time,17);
control=alignedSamples(out.AirbrakeControl,time,12);
diagnostics=alignedSamples(out.AirbrakeLQRDiagnostics,time,24);
weather=alignedSamples(out.WeatherTruth,time,11);
qTruth=scopeSamples(out.MC_Attitude,1,time,4);
qNav=scopeSamples(out.MC_Attitude,2,time,4);
pTruthNeu=scopeSamples(out.MC_Position,1,time,3);
pNavNeu=scopeSamples(out.MC_Position,2,time,3);
vTruthNeu=scopeSamples(out.MC_Velocity,1,time,3);
vNavNeu=scopeSamples(out.MC_Velocity,2,time,3);
bgNav=scopeSamples(out.MC_GyroBias,1,time,3);
bgTruth=scopeSamples(out.MC_GyroBias,2,time,3);
baNav=scopeSamples(out.MC_AccelBias,1,time,3);
baTruth=scopeSamples(out.MC_AccelBias,2,time,3);
allData=[truth,control,diagnostics,weather,qTruth,qNav,pTruthNeu,pNavNeu, ...
    vTruthNeu,vNavNeu,bgNav,bgTruth,baNav,baTruth];
assert(all(isfinite(allData(:))), ...
    'RocketMonteCarlo:Nonfinite','Required truth, Navigation or controller log is nonfinite.');
assert(all(weather(:,6)>0), ...
    'RocketMonteCarlo:SoundSpeed','Physical sound speed must be positive.');
target=vars.Airbrake.TargetApogeeAGL;
assert(isnumeric(target) && isscalar(target) && isreal(target) && ...
    isfinite(target) && target>0 && max(abs(control(:,8)-target))<1e-8, ...
    'RocketMonteCarlo:Target','Logged target must match the positive run target.');

row.Warnings=numel(out.SimulationMetadata.ExecutionInfo.WarningDiagnostics);
row.TargetAGL=target;
[row.ApogeeAGL,peakIndex]=max(-truth(:,3));
row.ApogeeErrorM=row.ApogeeAGL-target;
% Phase 2 is free flight and can still be ascending. It does not by itself
% establish apogee. Recovery phases, a recorded event, or an observed upward
% velocity zero crossing provide actual evidence that the peak was reached.
up=-truth(:,6);
crossedApogee=any(up(1:end-1)>0 & up(2:end)<=0 & ...
    truth(1:end-1,14)>=2 & truth(2:end,14)>=2);
eventTimes=truth(truth(:,15)>=0,15);
row.ApogeeObserved=~isempty(eventTimes) || any(truth(:,14)>=3) || crossedApogee;
if row.ApogeeObserved
    row.ApogeeTimeS=time(peakIndex);
    if ~isempty(eventTimes),row.ApogeeTimeS=eventTimes(1);end
end
row.TargetWithinTolerance=row.ApogeeObserved && ...
    abs(row.ApogeeErrorM)<=targetTolerance;
landingIndex=find(truth(:,14)==5,1);
row.FlightLanded=~isempty(landingIndex);
if row.FlightLanded
    row.Status="completed";
    row.LandingTimeS=time(landingIndex);
    row.LandingNorthM=truth(landingIndex,1);
    row.LandingEastM=truth(landingIndex,2);
    row.LandingRangeM=hypot(row.LandingNorthM,row.LandingEastM);
else
    row.Status="censored";
    row.Message="Simulation ended before landing.";
    if ~row.ApogeeObserved
        row.Message="Simulation ended before apogee and landing.";
    end
end
row.FinalNorthM=truth(end,1);row.FinalEastM=truth(end,2);
row.MaxSpeedMps=max(vecnorm(truth(:,4:6),2,2));
row.MaxMach=max(vecnorm(truth(:,4:6)-weather(:,1:3),2,2)./weather(:,6));
row.MaxBodyRateRadps=max(vecnorm(truth(:,11:13),2,2));
row.MaxActuatorPosition=max(control(:,11));
row.MaxActuatorRate=0;
if numel(time)>1
    row.MaxActuatorRate=max(abs(diff(control(:,11))./diff(time)));
end
valid=diagnostics(:,1)>.5;
row.GuidanceUpdates=nnz(any(diagnostics~=0,2));
row.RiccatiUpdates=nnz(valid & diagnostics(:,2)==1);
row.LowTargetUpdates=nnz(valid & diagnostics(:,2)==3);
row.HighTargetUpdates=nnz(valid & diagnostics(:,2)==2);

pNavNed=[pNavNeu(:,1:2),vars.LLA0(3)-pNavNeu(:,3)];
positionError=pNavNed-truth(:,1:3);
qTruthNorm=vecnorm(qTruth,2,2);qNavNorm=vecnorm(qNav,2,2);
assert(all(qTruthNorm>1e-12 & qNavNorm>1e-12), ...
    'RocketMonteCarlo:Quaternion','Truth and Navigation quaternion norms must be nonzero.');
quaternionDot=abs(sum((qTruth./qTruthNorm).*(qNav./qNavNorm),2));
attitudeError=rad2deg(2*acos(min(1,max(0,quaternionDot))));
postIgnition=time>=vars.Rocket.IgnitionTime;
if row.FlightLanded
    postIgnition=postIgnition & time<=time(landingIndex);
end
if any(postIgnition)
    positionNorm=vecnorm(positionError(postIgnition,:),2,2);
    row.MaxPositionErrorM=max(positionNorm);
    row.PositionRMSEM=sqrt(mean(positionNorm.^2));
    row.MaxAttitudeErrorDeg=max(attitudeError(postIgnition));
    row.AttitudeRMSEDeg=sqrt(mean(attitudeError(postIgnition).^2));
end
% Positive means Navigation's absolute Up exceeds the true height at peak.
row.PeakNavigationHeightErrorM=-positionError(peakIndex,3);
end

function row=blankRow(runId,elapsed)
row=struct('RunId',runId,'Status',"failed",'Message',"", ...
    'ElapsedSeconds',elapsed,'Warnings',NaN,'ApogeeAGL',NaN,'ApogeeTimeS',NaN, ...
    'TargetAGL',NaN,'ApogeeErrorM',NaN,'TargetWithinTolerance',false, ...
    'ApogeeObserved',false,'FlightLanded',false,'LandingTimeS',NaN, ...
    'LandingNorthM',NaN,'LandingEastM',NaN,'LandingRangeM',NaN, ...
    'FinalNorthM',NaN,'FinalEastM',NaN,'MaxSpeedMps',NaN,'MaxMach',NaN, ...
    'MaxBodyRateRadps',NaN,'MaxActuatorPosition',NaN,'MaxActuatorRate',NaN, ...
    'GuidanceUpdates',NaN,'RiccatiUpdates',NaN,'LowTargetUpdates',NaN, ...
    'HighTargetUpdates',NaN,'MaxPositionErrorM',NaN,'PositionRMSEM',NaN, ...
    'MaxAttitudeErrorDeg',NaN,'AttitudeRMSEDeg',NaN,'PeakNavigationHeightErrorM',NaN);
end

function data=scopeSamples(dataset,index,time,width)
assert(isa(dataset,'Simulink.SimulationData.Dataset') && numElements(dataset)>=index, ...
    'RocketMonteCarlo:Scope','Expected the configured Scope Dataset and signal order.');
signal=dataset{index};
data=alignedSamples(signal.Values,time,width);
end

function data=alignedSamples(log,time,width)
assert(isa(log,'timeseries') && ~isempty(log.Time), ...
    'RocketMonteCarlo:Log','Required logs must contain nonempty timeseries.');
sourceTime=double(log.Time(:));raw=double(log.Data);
assert(all(isfinite(sourceTime)) && all(diff(sourceTime)>=0), ...
    'RocketMonteCarlo:LogTime','Log timestamps must be finite and nondecreasing.');
if log.IsTimeFirst
    data=reshape(raw,numel(sourceTime),[]);
else
    data=reshape(permute(raw,[ndims(raw),1:ndims(raw)-1]),numel(sourceTime),[]);
end
assert(size(data,2)==width, ...
    'RocketMonteCarlo:LogWidth','Expected a %d-element log.',width);
[sourceTime,last]=unique(sourceTime,'last');data=data(last,:);
assert(sourceTime(1)<=time(1)+1e-9 && sourceTime(end)>=time(end)-1e-9, ...
    'RocketMonteCarlo:Coverage','Required logs must cover the truth time interval.');
if numel(sourceTime)==numel(time) && all(abs(sourceTime-time)<=1e-9),return;end
assert(numel(sourceTime)>1, ...
    'RocketMonteCarlo:Coverage','A single log sample cannot span the truth interval.');
data=interp1(sourceTime,data,time,'previous','extrap');
end
