function [hFig, hAx] = init3DPlot(hFig, xLimits, yLimits, zLimits, zones)
    %
    colormap = lines(3);

    % setup axes with manual limits
    hAx = axes('parent',hFig,'Color','white','Position',[0.05 0.05 0.9 0.9],'DataAspectRatio', [1 1 1]);
    hAx.XLim = xLimits;
    hAx.YLim = yLimits;
    hAx.ZLim = zLimits;
    hAx.XLimMode = 'manual';
    hAx.YLimMode = 'manual';
    hAx.ZLimMode = 'manual';
    hAx.XColor = colormap(1,:);
    hAx.YColor = colormap(2,:);
    hAx.ZColor = colormap(3,:);
    xlabel('X [m]'), ylabel('Y [m]'), zlabel('Z [m]')
        
    % draw reference lines for the axes at the origin
    refLineWidth = 1;
    line(hAx, xLimits, [0 0],'Color',hAx.XColor,'LineWidth', refLineWidth);
    line(hAx, [0 0], yLimits,'Color',hAx.YColor,'LineWidth', refLineWidth);
    line(hAx, [0 0], [0 0], zLimits,'Color',hAx.ZColor,'LineWidth', refLineWidth);
    
    clr = ['g', 'm', 'y'];
    hold on;
    for zoneIdx = 1:3
        zone = zones(zoneIdx);
       for cubeIdx = 1:3
           cube = zone.cubes(cubeIdx);
           plotCuboids(cube.x, cube.y, cube.z, clr(zoneIdx));
       end
    end
    
    hold off;
    % setup grid
    set(hAx, 'XGrid', 'on', 'YGrid', 'on', 'ZGrid', 'on', 'GridColor', [0 0 0])
    view(hAx,0,90)

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
    patch(x', z', y', color);
    alpha 0.5
end