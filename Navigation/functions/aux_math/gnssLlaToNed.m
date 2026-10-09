function positionNED = gnssLlaToNed(llaDegM, referenceLLArad)
%GNSSLLATONED Convert a GNSS LLA fix to the local navigation frame.
%   llaDegM is [latitude_deg; longitude_deg; absolute_altitude_m], 3-by-1.
%   referenceLLArad is [latitude_rad; longitude_rad; altitude_m], 3-by-1.
%   positionNED is [north_m; east_m; down_m], a 3-by-1 vector.
%
%   This is the inverse of the small-area WGS-84 curvature approximation
%   used by the Environment truth conversion. Longitude differences wrap
%   across the antimeridian. Down is reference altitude minus fix altitude.
%   The source altitude datum is unspecified; no geoid conversion is made.

    assert(isequal(size(llaDegM), [3, 1]));
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

    latitude_rad = llaDegM(1) * (pi / 180.0);
    longitude_rad = llaDegM(2) * (pi / 180.0);
    longitudeDifference_rad = mod(longitude_rad - longitude0_rad + pi, ...
        2.0 * pi) - pi;

    north_m = (latitude_rad - latitude0_rad) * ...
        (meridianRadius_m + altitude0_m);
    east_m = longitudeDifference_rad * ...
        ((primeVerticalRadius_m + altitude0_m) * cosLatitude0);
    down_m = altitude0_m - llaDegM(3);

    positionNED = [north_m; east_m; down_m];
end
