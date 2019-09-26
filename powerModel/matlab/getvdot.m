function vdot_in0 = getvdot(data)
% unpack cells
n = data{2,1};
% Estimate accelerations with finite difference of 200 Hz gps velocity data
vxdot = diff(data{28,1});
vydot = diff(data{29,1});
vzdot = -1*diff(data{30,1});

% Assume planar wind and level flight
vdot_in0 = zeros(n - 1, 3);

for i=1:n - 1
    vdot_in0(i,:) = [vxdot(i), vydot(i), vzdot(i)];
end

end