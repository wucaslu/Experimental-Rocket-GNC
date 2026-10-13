function rocketPlantValidateCdLookup(lookup)
%ROCKETPLANTVALIDATECDLOOKUP Validate an absolute grid or added-airbrake data.
% Rows are increasing nonnegative airspeeds; columns are control 0:0.1:1.
% Mode=1 selects scattered Samples=[deployment,Mach,addedCd]. CSV loading
% additionally rejects duplicate scattered coordinates before compilation.
%#codegen
if isstruct(lookup) && isscalar(lookup) && isfield(lookup,'Mode')
    if ~isnumeric(lookup.Mode) || ~isreal(lookup.Mode) || ...
            ~isscalar(lookup.Mode) || ~isfinite(lookup.Mode) || lookup.Mode~=1 || ...
            ~isfield(lookup,'Samples')
        error('RocketPlant:InvalidCdLookupTable','Scattered airbrake data requires Mode=1 and Samples.');
    end
    samples=lookup.Samples;
    if ~isnumeric(samples) || ~isreal(samples) || size(samples,2)~=3 || ...
            size(samples,1)<2 || any(~isfinite(samples(:))) || ...
            any(samples(:,1)<0) || ...
            any(samples(:,2)<0) || any(samples(:,3)<0)
        error('RocketPlant:InvalidCdLookupTable', ...
            'Samples must be finite nonnegative N-by-3 [deployment,Mach,addedCd], with N>=2.');
    end
    return
end
if ~isstruct(lookup) || ~isscalar(lookup) || ...
        ~isfield(lookup,'VelocityBreakpoints') || ...
        ~isfield(lookup,'ControlBreakpoints') || ~isfield(lookup,'Table')
    error('RocketPlant:InvalidCdLookupTable','Cd lookup must contain both axes and Table.');
end
v=lookup.VelocityBreakpoints;
u=lookup.ControlBreakpoints;
table=lookup.Table;
if ~isnumeric(v) || ~isreal(v) || ~iscolumn(v) || numel(v)<2 || ...
        any(~isfinite(v)) || any(v<0) || any(diff(double(v))<=0)
    error('RocketPlant:InvalidCdLookupTable', ...
        'VelocityBreakpoints must be a finite increasing nonnegative column with at least two rows.');
end
if ~isnumeric(u) || ~isreal(u) || ~isrow(u) || numel(u)~=11 || ...
        any(~isfinite(u)) || any(abs(double(u)-(0:0.1:1))>1e-12)
    error('RocketPlant:InvalidCdLookupTable','ControlBreakpoints must be the row 0:0.1:1.');
end
if ~isnumeric(table) || ~isreal(table) || ~isequal(size(table),[numel(v),11]) || ...
        any(~isfinite(table(:))) || any(table(:)<0)
    error('RocketPlant:InvalidCdLookupTable', ...
        'Table must be finite, nonnegative and sized number-of-speeds by 11 control levels.');
end
end
