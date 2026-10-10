function report = checkRocketCdLookupCsv(outputDirectory)
%CHECKROCKETCDLOOKUPCSV Check CSV schema, grid assembly and stateless reloads.
% Fixtures are shuffled samples of an independent bilinear Cd expression.
% Temporary CSVs are created in outputDirectory, or tempdir when omitted,
% and only those individually owned fixture files are deleted on completion.
if nargin<1 || isempty(outputDirectory), outputDirectory=''; end
navigationDirectory=fileparts(fileparts(mfilename('fullpath')));
previousPath=path;
pathCleanup=onCleanup(@() path(previousPath));
addpath(fullfile(navigationDirectory,'functions','aux_Rocket'));
fixtureDirectory=outputDirectory;
if isempty(fixtureDirectory), fixtureDirectory=tempdir; end
if ~isfolder(fixtureDirectory), mkdir(fixtureDirectory); end
names={'shuffledGridAndBilinearOracle','headersAndExtraColumns', ...
    'canonicalControlTolerance','duplicateAndIncompleteGrids', ...
    'invalidNumericValues','invalidHeadersAndColumnTypes', ...
    'missingFileAndFilenameTypes','freshReloadAfterFileChange'};
checks={@() checkGrid(fixtureDirectory),@() checkHeaders(fixtureDirectory), ...
    @() checkTolerance(fixtureDirectory),@() checkCompleteness(fixtureDirectory), ...
    @() checkNumeric(fixtureDirectory),@() checkSchema(fixtureDirectory), ...
    @() checkFilename(fixtureDirectory),@() checkReload(fixtureDirectory)};
results=repmat(struct('Name','','Passed',false,'ElapsedSeconds',0, ...
    'Metrics',struct(),'Message',''),numel(checks),1);
for k=1:numel(checks)
    results(k).Name=names{k};
    started=tic;
    try
        results(k).Metrics=checks{k}();
        results(k).Passed=true;
        fprintf('PASS %s\n',names{k});
    catch exception
        results(k).Message=exception.message;
        fprintf('FAIL %s: %s\n',names{k},exception.message);
    end
    results(k).ElapsedSeconds=toc(started);
end
report=struct('Passed',all([results.Passed]), ...
    'CreatedUTC',char(datetime('now','TimeZone','UTC')),'Checks',results);
if ~isempty(outputDirectory)
    writelines(string(jsonencode(report,PrettyPrint=true)), ...
        fullfile(outputDirectory,'rocket-cd-lookup-csv-checks.json'));
end
assert(report.Passed,'RocketCdCsvCheck:Failed','Cd lookup CSV checks failed: %s.', ...
    strjoin({results(~[results.Passed]).Name},', '));
end

function [source,velocity,level,expected]=fixture()
velocity=[0;37;120;260];
level=0:0.1:1;
[v,u]=ndgrid(velocity,level);
expected=coefficient(v,u);
source=table(expected(:),v(:),u(:),'VariableNames',{'Cd','velocity','level'});
% A fixed permutation covers every row without changing global RNG state.
order=mod((0:height(source)-1)*17+9,height(source))+1;
source=source(order,:);
end

function cd=coefficient(velocity,level)
cd=0.12+0.003*velocity+0.45*level+0.001*velocity.*level;
end

function metrics=checkGrid(directory)
[source,velocity,level,expected]=fixture();
[filename,cleanup]=ownedFile(directory); %#ok<ASGLU>
writetable(source,filename);
lookup=rocketPlantLoadCdLookupCsv(filename);
assert(isequal(fieldnames(lookup),{'VelocityBreakpoints';'ControlBreakpoints';'Table'}) && ...
    isequal(lookup.VelocityBreakpoints,velocity) && ...
    isequal(lookup.ControlBreakpoints,level) && ...
    max(abs(lookup.Table(:)-expected(:)))<1e-13, ...
    'RocketCdCsvCheck:Assembly','Shuffled CSV did not produce the required numeric grid.');
rocketPlantValidateCdLookup(lookup);
speeds=[0,18.5,60,190,260,350];
controls=[-0.4,0,0.05,0.35,0.72,1.4];
maximumError=0;
for v=speeds
    for u=controls
        usedV=min(velocity(end),max(velocity(1),v));
        usedU=min(1,max(0,u));
        [actual,query]=rocketPlantCdLookup(v,u,lookup);
        errorMagnitude=abs(actual-coefficient(usedV,usedU));
        maximumError=max(maximumError,errorMagnitude);
        assert(errorMagnitude<1e-12 && isequal(query,[usedV;usedU]), ...
            'RocketCdCsvCheck:Oracle','Loaded grid disagrees with the independent bilinear oracle.');
    end
end
metrics=struct('VelocitiesMps',velocity,'ControlNodes',level, ...
    'RowsLoaded',height(source),'QueriesChecked',numel(speeds)*numel(controls), ...
    'MaximumCdError',maximumError);
end

function metrics=checkHeaders(directory)
[source,velocity,level,expected]=fixture();
[filename,cleanup]=ownedFile(directory); %#ok<ASGLU>
% writecell preserves the actual header spelling and whitespace. Its quoted
% extra header exercises a comma inside a field while required order varies.
headers={' LEVEL ','note, "quoted"',' VELOCITY ',' cD '};
data=[num2cell(source.level),repmat({'metadata'},height(source),1), ...
    num2cell(source.velocity),num2cell(source.Cd)];
writecell([headers;data],filename,'Delimiter',',','QuoteStrings',true);
lookup=rocketPlantLoadCdLookupCsv(string(filename));
assert(isequal(lookup.VelocityBreakpoints,velocity) && ...
    isequal(lookup.ControlBreakpoints,level) && ...
    max(abs(lookup.Table(:)-expected(:)))<1e-13, ...
    'RocketCdCsvCheck:Headers','Reordered, trimmed, mixed-case headers or extra columns failed.');
metrics=struct('HeaderOrder',string(headers),'ExtraQuotedColumnAccepted',true);
end

function metrics=checkTolerance(directory)
[source,~,level,expected]=fixture();
[filename,cleanup]=ownedFile(directory); %#ok<ASGLU>
source.level=source.level+2e-10*sin((1:height(source))');
writetable(source,filename);
lookup=rocketPlantLoadCdLookupCsv(filename);
assert(isequal(lookup.ControlBreakpoints,level) && ...
    max(abs(lookup.Table(:)-expected(:)))<1e-13, ...
    'RocketCdCsvCheck:Canonical','Near-grid levels were not mapped to the canonical axis.');
% Two different raw levels that snap to the same node are still duplicates.
duplicate=source(1,:);
duplicate.level=duplicate.level+2e-10;
expectInvalid(directory,[source;duplicate]);
outside=source;
outside.level(1)=outside.level(1)+2e-9;
expectInvalid(directory,outside);
metrics=struct('AcceptedLevelPerturbation',2e-10, ...
    'SnappedDuplicateRejected',true,'OutsideToleranceRejected',true);
end

function metrics=checkCompleteness(directory)
source=fixture();
expectInvalid(directory,[source;source(1,:)]);
different=source(1,:);
different.Cd=different.Cd+0.2;
expectInvalid(directory,[source;different]);
expectInvalid(directory,source(2:end,:));
expectInvalid(directory,source(source.level~=0.5,:));
expectInvalid(directory,source(source.velocity==0,:));
metrics=struct('EqualAndUnequalDuplicatesRejected',true, ...
    'MissingPairAndWholeLevelRejected',true,'SingleVelocityRejected',true);
end

function metrics=checkNumeric(directory)
source=fixture();
columns={'Cd','velocity','level'};
badValues={NaN,Inf,-Inf};
count=0;
for column=1:numel(columns)
    for value=1:numel(badValues)
        malformed=source;
        malformed.(columns{column})(1)=badValues{value};
        expectInvalid(directory,malformed);
        count=count+1;
    end
end
for column=1:2
    malformed=source;
    malformed.(columns{column})(1)=-1;
    expectInvalid(directory,malformed);
    count=count+1;
end
malformed=source;
malformed.level(1)=0.055;
expectInvalid(directory,malformed);
malformed=source;
malformed.Cd(:)=0;
[filename,cleanup]=ownedFile(directory); %#ok<ASGLU>
writetable(malformed,filename);
lookup=rocketPlantLoadCdLookupCsv(filename);
assert(all(lookup.Table(:)==0),'RocketCdCsvCheck:Zero','Zero Cd must be valid.');
metrics=struct('NonfiniteAndNegativeCasesRejected',count, ...
    'OffGridControlRejected',true,'ZeroCdAccepted',true);
end

function metrics=checkSchema(directory)
source=fixture();
expectInvalid(directory,source(:,{'Cd','velocity'}));
malformed=source;
malformed.Properties.VariableNames{1}='coefficient';
expectInvalid(directory,malformed);
for column=1:3
    malformed=source;
    name=source.Properties.VariableNames{column};
    malformed.(name)=repmat("bad-number",height(source),1);
    expectInvalid(directory,malformed);
end
malformed=source;
malformed.Cd=string(source.Cd);
malformed.Cd(1)="0.2+0.1i";
expectInvalid(directory,malformed);
[filename,cleanup]=ownedFile(directory); %#ok<ASGLU>
rows=table2cell(source);
for headers={{'Cd','velocity','level','Cd'}, ...
        {'Cd','velocity','level',' CD '}}
    writecell([headers{1};[rows,num2cell(source.Cd)]],filename,'Delimiter',',');
    expectError(@() rocketPlantLoadCdLookupCsv(filename), ...
        'RocketPlant:InvalidCdLookupCsv');
end
writelines("",filename);
expectError(@() rocketPlantLoadCdLookupCsv(filename),'RocketPlant:InvalidCdLookupCsv');
writelines(["Cd,velocity,level";"0.3,10,0"],filename);
expectError(@() rocketPlantLoadCdLookupCsv(filename),'RocketPlant:InvalidCdLookupCsv');
metrics=struct('RequiredHeadersAndNumericTypesChecked',true, ...
    'IdenticalAndTrimmedDuplicateHeadersRejected',true,'EmptyFileRejected',true);
end

function metrics=checkFilename(directory)
[filename,cleanup]=ownedFile(directory); %#ok<ASGLU>
expectError(@() rocketPlantLoadCdLookupCsv(filename),'RocketPlant:CdLookupFileNotFound');
invalidNames={[],42,struct(),["one.csv","two.csv"],"",string(missing)};
for k=1:numel(invalidNames)
    expectError(@() rocketPlantLoadCdLookupCsv(invalidNames{k}), ...
        'RocketPlant:InvalidCdLookupCsv');
end
metrics=struct('MissingFileIdentifierChecked',true, ...
    'InvalidFilenameTypesChecked',numel(invalidNames));
end

function metrics=checkReload(directory)
source=fixture();
[filename,cleanup]=ownedFile(directory); %#ok<ASGLU>
writetable(source,filename);
before=rocketPlantLoadCdLookupCsv(filename);
source.Cd=source.Cd+0.7;
writetable(source,filename);
after=rocketPlantLoadCdLookupCsv(filename);
assert(max(abs(after.Table(:)-before.Table(:)-0.7))<1e-12, ...
    'RocketCdCsvCheck:Reload','Loading a changed filename returned stale coefficients.');
source(1,:)=[];
writetable(source,filename);
expectError(@() rocketPlantLoadCdLookupCsv(filename),'RocketPlant:InvalidCdLookupCsv');
metrics=struct('SameFilenameReloadedFresh',true,'SubsequentInvalidChangeRejected',true);
end

function expectInvalid(directory,source)
[filename,cleanup]=ownedFile(directory); %#ok<ASGLU>
writetable(source,filename);
expectError(@() rocketPlantLoadCdLookupCsv(filename),'RocketPlant:InvalidCdLookupCsv');
end

function expectError(action,identifier)
try
    action();
catch exception
    assert(strcmp(exception.identifier,identifier),'RocketCdCsvCheck:ErrorIdentifier', ...
        'Expected %s; got %s (%s).',identifier,exception.identifier,exception.message);
    return
end
error('RocketCdCsvCheck:MissingError','Expected error %s was not raised.',identifier);
end

function [filename,cleanup]=ownedFile(directory)
filename=[tempname(directory),'.csv'];
cleanup=onCleanup(@() deleteOwnedFile(filename));
end

function deleteOwnedFile(filename)
if isfile(filename), delete(filename); end
end
