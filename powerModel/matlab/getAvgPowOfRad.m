function powerWinAvg = getAvgPowOfRad(orbitPos, power, s, d)

rad = linspace(0, 2*pi, d);
powerWinAvg = zeros(size(rad))';

for i=1:length(rad)
    if (rad(i) - s < 0) || (rad(i) + s > 2*pi)
        idxs = (mod(rad(i) - s, 2*pi) <= orbitPos) |...
               (mod(rad(i) + s, 2*pi) >= orbitPos);
    else
        idxs = (mod(rad(i) - s, 2*pi) <= orbitPos) &...
               (mod(rad(i) + s, 2*pi) >= orbitPos);
    end
    
    powerWinAvg(i) = mean(power(idxs));
end
    
end

