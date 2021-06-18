function zoneMap = zoneAssign(numPoints, point3d, numZones, zone)

if (numPoints == 0)
    zoneMap = zeros(5, numZones);   
    xLoc = [];
    yLoc = [];
    zLoc = [];
    return;
end

xLoc = point3d(1, :);
yLoc = point3d(2, :);
zLoc = point3d(3, :);
zoneMap = zeros(numPoints, numZones);

for idx = 1:numZones
    cZone = zone(idx);

    for cIdx = 1:cZone.numCubes
        ind =       (xLoc > cZone.cubes(cIdx).x(1)) & ((xLoc < cZone.cubes(cIdx).x(2)));
        ind = ind & (yLoc > cZone.cubes(cIdx).y(1)) & ((yLoc < cZone.cubes(cIdx).y(2)));
        ind = ind & (zLoc > cZone.cubes(cIdx).z(1)) & ((zLoc < cZone.cubes(cIdx).z(2)));

        zoneMap(:, idx) = zoneMap(:, idx) | ind.';
    end
end
