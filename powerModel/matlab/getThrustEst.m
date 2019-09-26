function thrustEst = getThrustEst(data, params)
% unpack cells
n = data{2,1};
r = deg2rad(data{25,1});
p = deg2rad(data{26,1});
y = deg2rad(data{27,1});

% Assume planar wind and level flight
thrustEst = zeros(n, 3);

for i=1:n
    T = (params.m*params.g)/(cos(r(i))*cos(p(i)));
    R = eul2rotm([r(i) p(i) y(i)], 'XYZ');
    thrustEst(i,:) = [-T*sin(p(i)); T*sin(r(i)); params.m*params.g];
end

end