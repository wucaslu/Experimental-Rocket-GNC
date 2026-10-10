function rocketPlantValidateCdLookup(lookup)
%ROCKETPLANTVALIDATECDLOOKUP Validate an absolute speed/control Cd table.
% Rows are increasing nonnegative airspeeds; columns are control 0:0.1:1.
%#codegen
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
