function CdLookup = rocketPlantLoadCdLookupCsv(filename)
%ROCKETPLANTLOADCDLOOKUPCSV Load a complete absolute Cd grid from a CSV file.
% CdLookup = rocketPlantLoadCdLookupCsv(filename) reads the numeric columns
% Cd, velocity [m/s], and level. Headers are trimmed and matched without
% regard to case; column order and extra columns are unrestricted. Every
% velocity must have exactly one row for each level 0:0.1:1. Levels within
% 1e-9 of those nodes are mapped to the canonical control axis.
% The returned numeric struct is suitable for rocketPlantCdLookup. Call
% this host-side loader during initialization; it keeps no persistent state.
if ~(ischar(filename) && isrow(filename) && ~isempty(filename)) && ...
        ~(isstring(filename) && isscalar(filename) && ...
        ~ismissing(filename) && strlength(filename)>0)
    invalidCsv('Filename must be a nonempty character row or string scalar.');
end
if ~isfile(filename)
    error('RocketPlant:CdLookupFileNotFound', ...
        'Cd lookup CSV file does not exist: %s.',char(filename));
end
requiredHeaders=["cd","velocity","level"];
try
    % Table variable names must be unique even with the preserve rule. Check
    % the source header too, before readtable can rename duplicate headers.
    sourceHeaders=csvHeaders(readlines(filename));
    normalizedHeaders=lower(strtrim(sourceHeaders));
    for k=1:numel(requiredHeaders)
        if sum(normalizedHeaders==requiredHeaders(k))~=1
            invalidCsv('CSV must contain exactly one each of Cd, velocity, and level.');
        end
    end
    source=readtable(filename,'FileType','text','Delimiter',',', ...
        'DecimalSeparator','.','TextType','string', ...
        'VariableNamingRule','preserve','ReadVariableNames',true, ...
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
for k=1:numel(requiredHeaders)
    matches=find(importedHeaders==requiredHeaders(k));
    if numel(matches)~=1
        invalidCsv('CSV required headers could not be imported uniquely.');
    end
    columns(k)=matches;
end
values=cell(1,3);
for k=1:3
    values{k}=source{:,columns(k)};
    if ~isnumeric(values{k}) || ~isreal(values{k}) || ...
            ~iscolumn(values{k}) || any(~isfinite(values{k}))
        invalidCsv('Cd, velocity, and level must contain finite real numeric values.');
    end
    values{k}=double(values{k});
end
cd=values{1};
velocity=values{2};
level=values{3};
if any(cd<0) || any(velocity<0)
    invalidCsv('Cd and velocity must be nonnegative.');
end
[velocityBreakpoints,~,velocityIndex]=unique(velocity,'sorted');
if numel(velocityBreakpoints)<2
    invalidCsv('CSV must contain at least two distinct velocities.');
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
    invalidCsv('CSV contains a duplicate velocity-level pair.');
end
if any(pairCounts==0)
    invalidCsv('CSV is missing a velocity-level pair from the complete grid.');
end
cdTable=zeros(gridSize);
cdTable(linearIndex)=cd;
CdLookup=struct('VelocityBreakpoints',velocityBreakpoints, ...
    'ControlBreakpoints',controlBreakpoints,'Table',cdTable);
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
