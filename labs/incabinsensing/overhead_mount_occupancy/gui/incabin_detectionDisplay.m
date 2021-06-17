function incabin_detectionDisplay(figureTab, xLoc, yLoc, zLoc, tracker, zone, intBound)
%color = 'g';
%color2 = 'c';

%% front row
axes('parent', figureTab); 
subplot(1, 1, 1);
hold off;
cla;
scatter(xLoc, zLoc, 'm*','MarkerEdgeColor', 'b');
hold on;

grid on;
xlabel('x (m)');
ylabel('z (m)');

%Draw the interior boundary marked by the configuration
xlen = intBound.maxX - intBound.minX;
zlen = intBound.maxZ - intBound.minZ;
pos=[intBound.minX  intBound.minZ  xlen  zlen];
R = rectangle('Position',pos,'Curvature',[0.1 0.1], 'EdgeColor', [1 0 1]);

seatStr = {'Row 1 Driver','Row 1 Passenger','Row 2 Driver Side','Middle','Row 2 Passngr Side','','',''};
for idx = 1:length(tracker)
    if (tracker(idx).state)
        pos=[zone(idx).x_start zone(idx).z_start zone(idx).x_len zone(idx).z_len];
        if (tracker(idx).freeze)
            R = rectangle('Position',pos,'Curvature',[0.1 0.1],'FaceColor', [.75 0  0 .3], 'EdgeColor', [1 0 0]);
        else
            R = rectangle('Position',pos,'Curvature',[0.1 0.1],'FaceColor', [.5 .5 .5 .3], 'EdgeColor', [0 1 0]);
        end
        text('Position',[zone(idx).x_start+zone(idx).x_len zone(idx).z_start-0.05],'string',seatStr{idx})
    end
end

axis([-1.5, 1.5, -1.2, 1.2])
set(gca,'xdir','reverse','ydir','reverse')
hold off; 
