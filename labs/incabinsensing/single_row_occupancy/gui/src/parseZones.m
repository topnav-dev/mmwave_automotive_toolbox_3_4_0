function [zones] = parseZones(P)

zones = struct('numCubes', 0, 'cubes', struct ('x', 0, 'y', 0, 'z', 0));

cubeParams = P.cuboidDef;
i = 1;
for zoneNum = 1:3
    zones(zoneNum).numCubes = 3;
    for cubeNum = 1:3
        zones(zoneNum).cubes(cubeNum).x(1) = cubeParams.xMin(i);
        zones(zoneNum).cubes(cubeNum).x(2) = cubeParams.xMax(i);
        zones(zoneNum).cubes(cubeNum).y(1) = cubeParams.yMin(i);
        zones(zoneNum).cubes(cubeNum).y(2) = cubeParams.yMax(i);
        zones(zoneNum).cubes(cubeNum).z(1) = cubeParams.zMin(i);
        zones(zoneNum).cubes(cubeNum).z(2) = cubeParams.zMax(i);
        i = i+1;
    end
end