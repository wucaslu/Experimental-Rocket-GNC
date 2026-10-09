function llaDegM = gnssNeuToLla(positionNEU, referenceLLArad)
%GNSSNEUTOLLA Convert local trajectory position to GNSS latitude/longitude.
%   positionNEU is [north_m; east_m; absolute_altitude_m], a 3-by-1 vector.
%   referenceLLArad is [latitude_rad; longitude_rad; altitude_m], 3-by-1.
%   llaDegM is [latitude_deg; longitude_deg; absolute_altitude_m], 3-by-1.
%
%   This small-area WGS-84 curvature approximation matches the existing
%   Environment truth conversion. The third position component is absolute
%   source altitude, not local Up, so the reference altitude is not added.
%   The source altitude datum is unspecified; no geoid conversion is made.

    assert(isequal(size(positionNEU), [3, 1]));
    assert(isequal(size(referenceLLArad), [3, 1]));

    semimajorAxis_m = 6378137.0;
    eccentricitySquared = 6.69437999014e-3;
    latitude0_rad = referenceLLArad(1);
    longitude0_rad = referenceLLArad(2);
    altitude0_m = referenceLLArad(3);
    cosLatitude0 = cos(latitude0_rad);

    % Longitude is undefined at a reference pole in this local mapping.
    assert(abs(cosLatitude0) > 1.0e-12);
    sinLatitude0 = sin(latitude0_rad);
    denominator = 1.0 - eccentricitySquared * sinLatitude0^2;
    primeVerticalRadius_m = semimajorAxis_m / sqrt(denominator);
    meridianRadius_m = semimajorAxis_m * (1.0 - eccentricitySquared) / ...
        denominator^(3.0 / 2.0);

    latitude_rad = latitude0_rad + ...
        positionNEU(1) / (meridianRadius_m + altitude0_m);
    longitude_rad = longitude0_rad + positionNEU(2) / ...
        ((primeVerticalRadius_m + altitude0_m) * cosLatitude0);
    longitude_rad = mod(longitude_rad + pi, 2.0 * pi) - pi;

    llaDegM = [latitude_rad * (180.0 / pi); ...
               longitude_rad * (180.0 / pi); positionNEU(3)];
end
