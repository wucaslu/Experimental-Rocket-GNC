function [cdCommand,enabled,info] = rocketPlantCdLookupCommand(velocityNEU,windNED,controlLevel,lookupEnabled,manualCd,manualEnabled,lookup)
%ROCKETPLANTCDLOOKUPCOMMAND Airspeed lookup and explicit Cd source selection.
% Priority: manual override, lookup override, nominal powered/coast model.
% NEU velocity becomes NED before subtracting NED wind. Inputs are current
% truth signals, independent of sensor errors and GNSS velocity measurements.
% info=[airspeed;usedSpeed;rawControl;usedControl;lookupCd;lookupEnable;mode].
% mode: 0 nominal, 1 lookup, 2 manual. No persistent state.
if ~isnumeric(velocityNEU) || ~isreal(velocityNEU) || ...
        ~isequal(size(velocityNEU),[3 1]) || any(~isfinite(velocityNEU)) || ...
        ~isnumeric(windNED) || ~isreal(windNED) || ...
        ~isequal(size(windNED),[3 1]) || any(~isfinite(windNED))
    error('RocketPlant:InvalidCdLookupInput','Velocity and wind must be finite real numeric 3-by-1 columns.');
end
[cdCommand,enabled]=rocketPlantValidateCdControl(manualCd,manualEnabled);
[~,lookupEnabled]=rocketPlantValidateCdControl(0,lookupEnabled);
velocityNED=double(velocityNEU).*[1;1;-1];
speed=norm(velocityNED-double(windNED));
[lookupCd,query]=rocketPlantCdLookup(speed,controlLevel,lookup);
mode=0;
if enabled
    mode=2;
elseif lookupEnabled
    cdCommand=lookupCd;
    enabled=true;
    mode=1;
end
info=[speed;query(1);double(controlLevel);query(2);lookupCd;double(lookupEnabled);mode];
end
