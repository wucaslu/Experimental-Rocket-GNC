function [pHat, dP_dD, dP_ds, logRatio] = baroScalePressure(Down, h0_m, pPad, s)
%BAROSCALEPRESSURE Pressure profile with an estimated atmospheric scale.
% Inputs: local NED Down position [m], origin altitude h0_m above mean sea
% level [m], held measured launch pressure pPad [Pa] and positive dimensionless
% scale s. Pressure and its pad reference must use the same sensor calibration.
% Outputs: predicted pressure [Pa], derivatives with respect to Down [Pa/m]
% and s [Pa], and dimensionless ISA pressure log ratio. The profile uses only
% navigation states and the held launch reference, with no weather/truth input.

[pISA, dISA_dD] = ISA_model(Down, h0_m);
[pISAorigin, ~] = ISA_model(0, h0_m);
logRatio = log(pISA / pISAorigin);
pHat = pPad * exp(s * logRatio);
dP_dD = pHat * s * dISA_dD / pISA;
dP_ds = pHat * logRatio;
end
