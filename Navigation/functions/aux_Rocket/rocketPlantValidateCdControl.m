function [cdCommand,enabled] = rocketPlantValidateCdControl(cdCommand,enabled)
%ROCKETPLANTVALIDATECDCONTROL Validate the external absolute body-Cd override.
% Command is dimensionless, finite, real, scalar and nonnegative. Enable is
% logical or numeric 0/1. Convert numeric classes before doing arithmetic.
%#codegen
if ~isnumeric(cdCommand) || ~isscalar(cdCommand) || ~isreal(cdCommand) || ...
        ~isfinite(cdCommand) || cdCommand<0
    error('RocketPlant:InvalidCdControl', ...
        'Cd_command must be a finite, real, nonnegative numeric scalar.');
end
if ~(isnumeric(enabled) || islogical(enabled)) || ~isscalar(enabled) || ...
        ~isreal(enabled) || ~isfinite(enabled) || ~(enabled==0 || enabled==1)
    error('RocketPlant:InvalidCdControl', ...
        'Cd_override_enabled must be a logical scalar or numeric 0 or 1.');
end
cdCommand=double(cdCommand);
enabled=logical(enabled);
end
