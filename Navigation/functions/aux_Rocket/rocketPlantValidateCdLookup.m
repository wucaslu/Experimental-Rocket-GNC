function rocketPlantValidateCdLookup(lookup)
%ROCKETPLANTVALIDATECDLOOKUP Validate total-Cd grids or added-airbrake data.
% Input: lookup is a scalar numeric configuration struct. A grid without
% Mode uses increasing nonnegative VelocityBreakpoints [m/s]. Mode=2 uses
% increasing nonnegative MachBreakpoints and stores total Cd. Both grids
% have ControlBreakpoints=0:0.1:1 and Table sized axis rows by 11 columns.
% Mode=1 selects scattered Samples=[deployment,Mach,addedCd]. CSV loading
% additionally rejects duplicate scattered coordinates before compilation.
% Output: none; invalid schemas or numeric values raise a named error.
if ~isstruct(lookup) || ~isscalar(lookup)
    error('RocketPlant:InvalidCdLookupTable','Cd lookup must be a scalar struct.');
    return
end
if isfield(lookup,'Mode')
    if ~isnumeric(lookup.Mode) || ~isreal(lookup.Mode) || ...
            ~isscalar(lookup.Mode) || ~isfinite(lookup.Mode) || ...
            (lookup.Mode~=1 && lookup.Mode~=2)
        error('RocketPlant:InvalidCdLookupTable','Mode must be the finite real scalar 1 or 2.');
    end
    % Field guards also let Coder specialize fixed-schema structs without
    % requiring unused Samples or VelocityBreakpoints fields in Mode=2.
    if isfield(lookup,'Samples')
        if lookup.Mode~=1
            error('RocketPlant:InvalidCdLookupTable','Samples requires Mode=1 for added-airbrake data.');
        end
        samples=lookup.Samples;
        if ~isnumeric(samples) || ~isreal(samples) || ~ismatrix(samples) || size(samples,2)~=3 || ...
                size(samples,1)<2 || any(~isfinite(samples(:))) || ...
                any(samples(:,1)<0) || ...
                any(samples(:,2)<0) || any(samples(:,3)<0)
            error('RocketPlant:InvalidCdLookupTable', ...
                'Samples must be finite nonnegative N-by-3 [deployment,Mach,addedCd], with N>=2.');
        end
        return
    elseif isfield(lookup,'MachBreakpoints')
        if lookup.Mode~=2
            error('RocketPlant:InvalidCdLookupTable','MachBreakpoints requires Mode=2 for total Cd.');
        end
        v=lookup.MachBreakpoints;
    else
        error('RocketPlant:InvalidCdLookupTable','Mode=1 requires Samples; Mode=2 requires MachBreakpoints.');
        return
    end
elseif isfield(lookup,'VelocityBreakpoints')
    v=lookup.VelocityBreakpoints;
else
    error('RocketPlant:InvalidCdLookupTable','A total-Cd grid without Mode requires VelocityBreakpoints.');
    return
end
if isfield(lookup,'ControlBreakpoints') && isfield(lookup,'Table')
    u=lookup.ControlBreakpoints;
    table=lookup.Table;
else
    error('RocketPlant:InvalidCdLookupTable','Cd grid must contain ControlBreakpoints and Table.');
    return
end
if ~isnumeric(v) || ~isreal(v) || ~iscolumn(v) || numel(v)<2 || ...
        any(~isfinite(v)) || any(v<0) || any(diff(double(v))<=0)
    error('RocketPlant:InvalidCdLookupTable', ...
        'VelocityBreakpoints or MachBreakpoints must be a finite increasing nonnegative column with at least two rows.');
end
if ~isnumeric(u) || ~isreal(u) || ~isrow(u) || numel(u)~=11 || ...
        any(~isfinite(u)) || any(abs(double(u)-(0:0.1:1))>1e-12)
    error('RocketPlant:InvalidCdLookupTable','ControlBreakpoints must be the row 0:0.1:1.');
end
if ~isnumeric(table) || ~isreal(table) || ~isequal(size(table),[numel(v),11]) || ...
        any(~isfinite(table(:))) || any(table(:)<0)
    error('RocketPlant:InvalidCdLookupTable', ...
        'Table must be finite, nonnegative and sized number-of-axis-nodes by 11 control levels.');
end
end
