% Display cartoon images of occupants in various zone positions
function car_display(figureTab, tracker, numZones, xscale, seat_pos, frameIdx)

    global car_bg;
    global person;

    xstart = (1.0 - xscale) / 2;

    ha = axes('parent', figureTab, 'units', 'normalized', 'position', [xstart 0 xscale 1]);
%    subplot(1, 1, 1);


    if (numZones <= 2)
        numRows = 1;
    else
        numRows = 2;
    end

    frameFlag = bitand(frameIdx, 3);

    if (0)
        % This creates the 'background' axes
        ha = axes('units','normalized', 'position',[0 0 1 1]);
        % Move the background axes to the bottom
        uistack(ha,'bottom');
        % Load in a background image and display it using the correct colors
        hi = imagesc(car_bg);

        %colormap gray
        % Turn the handlevisibility off so that we don't inadvertently plot into the axes again
        % Also, make the axes invisible
        set(ha,'handlevisibility','off', 'visible','off');
        hold on;
    end

hold on;

    if (frameFlag == 0)
        %Front driver zone
        subplot('Position', seat_pos(1,:));
        cla(subplot('Position', seat_pos(1,:)));
%        set(gca,'YTick',[],'XTick',[]);
        set(gca,'visible', 'off', 'YTick',[],'XTick',[]);
        set(gca,'Color','none');

        xlabel('');
        ylabel('');

        if (tracker(1).state)
        % subplot positions are left, bottom, width, height and are normalized 0 to 1.0
           imshow(person, 'Border','tight');
    %    ax.Visible = "off";
        end

        %Front passenger zone
        subplot('Position', seat_pos(2,:)); 
        cla(subplot('Position', seat_pos(2,:)));
        set(gca,'visible', 'off', 'YTick',[],'XTick',[]);
        set(gca,'Color','none');

        xlabel('');
        ylabel('');

        if (tracker(2).state)
            imshow(person, 'Border','tight');
        end

    elseif ((frameFlag == 2) & (numRows > 1))
        %left back zone
        subplot('Position', seat_pos(3,:));
        cla(subplot('Position', seat_pos(3,:)));
        set(gca,'visible', 'off', 'YTick',[],'XTick',[]);
        set(gca,'Color','none');

        xlabel('');
        ylabel('');

        if (tracker(3).state)
           imshow(person, 'Border','tight');
        end

        %center zone
        subplot('Position', seat_pos(4,:)); 
        cla(subplot('Position', seat_pos(4,:)));
        set(gca,'visible', 'off', 'YTick',[],'XTick',[]);
        set(gca,'Color','none');

        xlabel('');
        ylabel('');

        if (tracker(4).state)
           imshow(person, 'Border','tight');
        end

        %right back zone
        subplot('Position', seat_pos(5,:)); 
        cla(subplot('Position', seat_pos(5,:)));
        set(gca,'visible', 'off', 'YTick',[],'XTick',[]);
        set(gca,'Color','none');

        xlabel('');
        ylabel('');

        if (tracker(5).state)
            imshow(person, 'Border','tight');
        end
    else
        subplot('Position', seat_pos(1,:)); 
    end
hold off;
return
    