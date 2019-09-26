function [rx, ry] = GPStoRadius(latH, lonH, lat, lon)
% radius of the earth
R = 6.378137e3; % km

% Because of the relatively small length scale we approximate the distance
% with a simply arc length
rx = (lon*pi/180 - lonH*pi/180)*R*1000;
ry = (lat*pi/180 - latH*pi/180)*R*1000;
end

