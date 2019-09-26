function v_inf = getv_inf(data)
% unpack cells
wx = data{13,1};
wy = data{14,1};
vx = data{21,1};
vy = data{20,1};

% Assume planar wind and level flight
vz = zeros(size(vx));

v_inf = [vx+wx, vy+wy, vz];
end

