function [zoneMap, xLoc, yLoc, zLoc] = zone_assign(numPoints, point3d, numZones, zone)

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

    for cIdx = 1:cZone.numCuboids
        ind =       (xLoc > cZone.cuboid(cIdx).x(1)) & ((xLoc < cZone.cuboid(cIdx).x(2)));
        ind = ind & (yLoc > cZone.cuboid(cIdx).y(1)) & ((yLoc < cZone.cuboid(cIdx).y(2)));
        ind = ind & (zLoc > cZone.cuboid(cIdx).z(1)) & ((zLoc < cZone.cuboid(cIdx).z(2)));

        zoneMap(:, idx) = zoneMap(:, idx) | ind.';
    end
end
