function [orbitPos, power] = getPowerVsOrbitPos(fullFlt,windHeading)
% Get starting and ending indexes
startIdx = fullFlt{4, 1};
endIdx = fullFlt{5, 1};

% Down sample power measurements
power = fullFlt{10, 1}(startIdx:200:endIdx);

% Because of the relatively small length scale we approximate the distance
% with a simple arc length between the gps home point and current gps
% location instead of using somehting more involved such as the haversine
% formula.
homePt = fullFlt{6, 1};
lat = fullFlt{7, 1}(startIdx:200:endIdx);
lon = fullFlt{8, 1}(startIdx:200:endIdx);
rx = zeros(size(lat));
ry = zeros(size(lon));
r = zeros(size(rx));
orbitPos = zeros(size(rx));

for i=1:length(rx)
    [rx(i), ry(i)] = GPStoRadius(homePt(1), homePt(2), lat(i), lon(i));
    r(i) = sqrt(rx(i)^2 + ry(i)^2);
    if ry(i) < 0
        orbitPos(i) = 2*pi - acos(rx(i)/r(i));
    else
        orbitPos(i) = acos(rx(i)/r(i));
    end
    orbitPos(i) = mod(orbitPos(i)- windHeading, 2*pi);
end

end

