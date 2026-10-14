function CdLookup = rocketPlantLoadCdLookupCsv(filename,schema)
%ROCKETPLANTLOADCDLOOKUPCSV Load a total-Cd grid or added-airbrake samples.
% CdLookup = rocketPlantLoadCdLookupCsv(filename) selects a schema from
% named CSV headers, matched without regard to case or surrounding spaces:
%   Cd, velocity, level: total Cd on an airspeed [m/s]/control grid.
%   Level, Mach, Cd: total Cd on a dimensionless Mach/control grid (Mode=2).
%   deployment_level, mach, cd: scattered added Cd samples (Mode=1).
% Headered columns may appear in any order; extra columns are permitted.
%
% CdLookup = rocketPlantLoadCdLookupCsv(filename,'level-mach-cd') also
% accepts a headerless CSV whose three columns are [Level,Mach,Cd], in that
% order. Headerless column meanings are never inferred automatically.
% The optional schema can alternatively be 'cd-velocity-level',
% 'deployment-level-mach-cd', or 'auto' (the default). Explicit headerless
% files must contain exactly three numeric columns in their schema order.
%
% Inputs: filename is the CSV path; schema specifies its column meanings.
% Output: a numeric struct suitable for rocketPlantCdLookup. Total-Cd grids
% contain one row for every axis/control pair, with control nodes 0:0.1:1.
% Mach grids have Mode=2, MachBreakpoints (column), ControlBreakpoints (row)
% and Table (Mach rows, control columns). Levels within 1e-9 of control
% nodes are mapped to the canonical axis. Mode=1 preserves scattered
% Samples=[deployment_level,mach,addedCd] for RocketPy's Shepard method.
% Call this host-side loader during initialization; it has no hidden state.
if nargin<2, schema='auto'; end
if ~(ischar(filename) && isrow(filename) && ~isempty(filename)) && ...
        ~(isstring(filename) && isscalar(filename) && ...
        ~ismissing(filename) && strlength(filename)>0)
    invalidCsv('Filename must be a nonempty character row or string scalar.');
end
if ~isfile(filename)
    error('RocketPlant:CdLookupFileNotFound', ...
        'Cd lookup CSV file does not exist: %s.',char(filename));
end
if ~(ischar(schema) && isrow(schema) && ~isempty(schema)) && ...
        ~(isstring(schema) && isscalar(schema) && ...
        ~ismissing(schema) && strlength(schema)>0)
    invalidCsv('Schema must be a nonempty character row or string scalar.');
end
schema=lower(strtrim(string(schema)));
if ~ismember(schema,["auto","cd-velocity-level","level-mach-cd", ...
        "deployment-level-mach-cd"])
    invalidCsv('Unsupported schema; use auto, cd-velocity-level, level-mach-cd or deployment-level-mach-cd.');
end
requiredHeaders=["cd","velocity","level"];
scatteredSource=false;
machGrid=false;
headerless=false;
try
    % Table variable names must be unique even with the preserve rule. Check
    % the source header too, before readtable can rename duplicate headers.
    sourceHeaders=csvHeaders(readlines(filename));
    normalizedHeaders=lower(strtrim(sourceHeaders));
    hasVelocityHeaders=all(ismember(requiredHeaders,normalizedHeaders));
    hasAbsoluteMachHeaders=all(ismember(["cd","mach","level"],normalizedHeaders));
    hasScatteredHeaders=all(ismember(["cd","mach","deployment_level"],normalizedHeaders));
    if sum([hasVelocityHeaders,hasAbsoluteMachHeaders,hasScatteredHeaders])>1
        invalidCsv('CSV headers must select exactly one total-Cd or added-airbrake schema.');
    end
    if schema=="auto"
        if hasScatteredHeaders
            schema="deployment-level-mach-cd";
        elseif hasAbsoluteMachHeaders
            schema="level-mach-cd";
        elseif hasVelocityHeaders
            schema="cd-velocity-level";
        else
            invalidCsv('CSV needs named lookup headers; select an explicit schema for headerless data.');
        end
    end
    if schema=="deployment-level-mach-cd"
        requiredHeaders=["cd","mach","deployment_level"];
        scatteredSource=true;
        rawColumns=[3,2,1];
    elseif schema=="level-mach-cd"
        requiredHeaders=["cd","mach","level"];
        machGrid=true;
        rawColumns=[3,2,1];
    else
        rawColumns=[1,2,3];
    end
    if all(ismember(requiredHeaders,normalizedHeaders))
        for k=1:numel(requiredHeaders)
            if sum(normalizedHeaders==requiredHeaders(k))~=1
                invalidCsv('CSV must contain exactly one of each header for its Cd lookup schema.');
            end
        end
    elseif numel(sourceHeaders)==3 && all(isfinite(str2double(sourceHeaders)))
        headerless=true;
    else
        invalidCsv('CSV must have the selected headers or exactly three numeric headerless columns.');
    end
    source=readtable(filename,'FileType','text','Delimiter',',', ...
        'DecimalSeparator','.','TextType','string', ...
        'VariableNamingRule','preserve','ReadVariableNames',~headerless, ...
        'NumHeaderLines',0);
catch exception
    if strcmp(exception.identifier,'RocketPlant:InvalidCdLookupCsv')
        rethrow(exception);
    end
    error('RocketPlant:InvalidCdLookupCsv', ...
        'Cannot read Cd lookup CSV "%s": %s',char(filename),exception.message);
end
importedHeaders=lower(strtrim(string(source.Properties.VariableNames)));
columns=zeros(1,3);
if headerless
    if width(source)~=3
        invalidCsv('Headerless lookup CSV must contain exactly three columns.');
    end
    columns=rawColumns;
else
    for k=1:numel(requiredHeaders)
        matches=find(importedHeaders==requiredHeaders(k));
        if numel(matches)~=1
            invalidCsv('CSV required headers could not be imported uniquely.');
        end
        columns(k)=matches;
    end
end
values=cell(1,3);
for k=1:3
    values{k}=source{:,columns(k)};
    if ~isnumeric(values{k}) || ~isreal(values{k}) || ...
            ~iscolumn(values{k}) || any(~isfinite(values{k}))
        invalidCsv('All three lookup columns must contain finite real numeric values.');
    end
    values{k}=double(values{k});
end
cd=values{1};
velocity=values{2};
level=values{3};
if scatteredSource
    % The published table includes deployment=1.1 support points. Keep
    % those samples: the actuator/query, rather than the source, is clamped.
    if numel(cd)<2 || any(cd<0) || any(velocity<0) || any(level<0)
        invalidCsv('Airbrake data needs at least two samples and nonnegative deployment/Mach/Cd.');
    end
    if size(unique([level,velocity],'rows'),1)~=numel(level)
        invalidCsv('CSV contains a duplicate deployment-Mach pair.');
    end
    CdLookup=struct('Mode',1,'Samples',[level,velocity,cd]);
    rocketPlantValidateCdLookup(CdLookup);
    return
end
if any(cd<0) || any(velocity<0)
    invalidCsv('Cd and the velocity/Mach axis must be nonnegative.');
end
[velocityBreakpoints,~,velocityIndex]=unique(velocity,'sorted');
if numel(velocityBreakpoints)<2
    invalidCsv('CSV must contain at least two distinct velocity/Mach nodes.');
end
controlBreakpoints=0:0.1:1;
[levelDistance,controlIndex]=min(abs(level-controlBreakpoints),[],2);
if any(levelDistance>1e-9)
    invalidCsv('Every level must be within 1e-9 of a control node 0:0.1:1.');
end
gridSize=[numel(velocityBreakpoints),numel(controlBreakpoints)];
linearIndex=sub2ind(gridSize,velocityIndex,controlIndex);
pairCounts=accumarray(linearIndex,1,[prod(gridSize),1]);
if any(pairCounts>1)
    invalidCsv('CSV contains a duplicate velocity/Mach-level pair.');
end
if any(pairCounts==0)
    invalidCsv('CSV is missing a velocity/Mach-level pair from the complete grid.');
end
cdTable=zeros(gridSize);
cdTable(linearIndex)=cd;
if machGrid
    CdLookup=struct('Mode',2,'MachBreakpoints',velocityBreakpoints, ...
        'ControlBreakpoints',controlBreakpoints,'Table',cdTable);
else
    CdLookup=struct('VelocityBreakpoints',velocityBreakpoints, ...
        'ControlBreakpoints',controlBreakpoints,'Table',cdTable);
end
rocketPlantValidateCdLookup(CdLookup);
end

function headers=csvHeaders(lines)
% Parse the first CSV record, including quoted extra headers and escaped
% quotes. Checking its original names catches readtable's name uniquification.
if isempty(lines)
    invalidCsv('CSV is empty.');
end
headers=strings(1,0);
token='';
inQuotes=false;
for row=1:numel(lines)
    line=char(lines(row));
    if row==1 && ~isempty(line) && line(1)==char(65279)
        line(1)=[];
    end
    column=1;
    while column<=numel(line)
        character=line(column);
        if character=='"'
            if inQuotes && column<numel(line) && line(column+1)=='"'
                token(end+1)='"'; %#ok<AGROW>
                column=column+1;
            else
                inQuotes=~inQuotes;
            end
        elseif character==',' && ~inQuotes
            headers(end+1)=string(token); %#ok<AGROW>
            token='';
        else
            token(end+1)=character; %#ok<AGROW>
        end
        column=column+1;
    end
    if ~inQuotes
        headers(end+1)=string(token); %#ok<AGROW>
        return
    end
    token(end+1)=newline; %#ok<AGROW>
end
invalidCsv('CSV header has an unterminated quoted field.');
end

function invalidCsv(message)
error('RocketPlant:InvalidCdLookupCsv','%s',message);
end
