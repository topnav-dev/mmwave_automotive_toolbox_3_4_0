function zone_display(figureTab, numZones, zone, xLoc, yLoc, zLoc, scene, row)

    AZ = -30; EL = 20; 
    clr = ['c', 'g', 'g', 'm', 'y'];
    
    %% front row
    %figure(figureID);
    axes('parent', figureTab);
    subplot(1, 1, 1);
    cla;
    view(AZ,EL);

    if (row == 1)
       % front driver and passenger side
       hold on;
       for zIdx = 1:2
        cZone = zone(zIdx);
        for cIdx = 1:cZone.numCuboids
            plotCuboids(cZone.cuboid(cIdx).x, cZone.cuboid(cIdx).y, cZone.cuboid(cIdx).z, clr(zIdx));
        end
       end

       axis([-1.5, 1.5, -1.2, 1.2, 0, 1.5])
       xlabel('x');
       ylabel('z');
       zlabel('y');
       scatter3(xLoc, zLoc, yLoc, 'filled','bo');
       title('First row seats')
       hold off;

    else
       % second row driver, middle and passenger side
       hold on;
       for zIdx = 3:5
           cZone = zone(zIdx);
           for cIdx = 1:cZone.numCuboids
               plotCuboids(cZone.cuboid(cIdx).x, cZone.cuboid(cIdx).y, cZone.cuboid(cIdx).z, clr(zIdx));
           end
       end

       axis([-1.5, 1.5, -1.2, 1.2, 0, 1.5])
       xlabel('x');
       ylabel('z');
       zlabel('y');
       scatter3(xLoc, zLoc, yLoc, 'filled','bo');
       title('Second row seats')
       hold off;
    end
end


function plotCuboids(xin, yin, zin, color)

    a = -pi : pi/2 : pi;                      % Define Corners
    ph = pi/4;

    t1 = (xin(1) + xin(2))/2;  %min and max X
    t2 = (xin(2) - xin(1))/2;
    x = t1 + t2*[cos(a+ph); cos(a+ph)]/cos(ph);

    t1 = (yin(1) + yin(2))/2;  %min and max Y
    t2 = (yin(2) - yin(1))/2;
    y = t1 + t2*[sin(a+ph); sin(a+ph)]/sin(ph);

    t1 = (zin(1) + zin(2))/2;  %min and max Z
    t2 = (zin(2) - zin(1))/2;
    z = t1 + t2*[-ones(size(a)); ones(size(a))];

    surf(x, z, y, 'FaceColor',color);
%    hold on;
end