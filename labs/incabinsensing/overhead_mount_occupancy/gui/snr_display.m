function snr_display(figureTab, tracker, numZones, overload)

    axes('parent', figureTab);
    subplot(1, 1, 1);
    cla;

    pos = get(gcf, 'Position');
    figure_width = pos(3);
    num_col = (numZones * 3 - 1);
    col_size = floor(figure_width / num_col);
    maxHght = 100;
    maxPow = overload;
    midPow = overload * 0.90;
    lowPow = overload * 0.10;

    mainRect = ones(maxHght, num_col * col_size);
    imagesc(mainRect);
    
    colormap(gca, [0.75 0.75 0.75]);

    xlabel('');
    ylabel('');
    set(gca,'YTickLabel',[]);
    set(gca,'XTickLabel',[]);
    title('Average SNR by Zone');

    for idx = 1:numZones
        zpower = tracker(idx).avgSnr;
        if (zpower < maxPow)
          dispPow = zpower;
        else
          dispPow = maxPow;
        end

        if (zpower < lowPow)
            fclr = 'y';
        elseif (zpower < midPow)
            fclr = 'g';
        else
            fclr = 'r';
        end
        
        x = (idx-1)*col_size*3;
        h = maxHght / maxPow * dispPow;
        if (h < 2)
          h = 2;
        end
        y = maxHght - h + 1;

        barRect = [x y (col_size*2) h];
        rectangle('Position', barRect, 'FaceColor', fclr, 'LineWidth', 1.0);

        ztext = sprintf('%.1f', zpower);
        xpos = x / figure_width + 0.01;
        text(xpos, 0.03, ztext, 'FontSize', 18, 'Units', 'normalized');
    end
return
