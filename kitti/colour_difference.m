function colour_difference(img, velo_img, velo)
    % make sure you are not thresholding for height when using
% Finds 'objects' based on adjacent depth colours before and after 'object'
% Analogy - find the vertical lines:
% ------|||||||---------
% By looking at the adjacent horizontal bars (i.e. adjacent horizontal similar colours), you can determine that there is something in between them.

% Parameters:
% img               | image to plot on
% velo_img          | LiDAR point cloud projected onto img
% velo              | LiDAR point cloud
% opt_grad_thresh   | optional gradient threshold (defaults to 2)

% % 12 starts losing detail in car a bit
% for loop cannot modify loop variable inside of itself
% use a while instead
% colour_thresh = 12;
bg_points = uint8(zeros(size(img)));
for colour_thresh = 8:11
  i = 1;
  while ( i < size(velo_img, 1))
    % i = i + 1;
    if (outside_image(img, velo_img, i))
      i = i + 1;
      continue;
    end
    starting_colour = round(64*5/velo(i,1));
    current_colour = starting_colour;
    plot(velo_img(i,1),velo_img(i,2),'x','LineWidth',4, 'Color', 'g');
    while (abs(current_colour - starting_colour) < colour_thresh)
      current_colour = round(64*5/velo(i, 1));
      i = i + 1;
      if (outside_image(i))
        break;
      end
    end

    idx_2 = i;
    plot(velo_img(i,1),velo_img(i,2),'x','LineWidth',4, 'Color', 'y');
    starting_colour_2 = round(64*5/velo(i, 1));

    while (abs(current_colour - starting_colour_2) < colour_thresh)
      current_colour = round(64*5/velo(i, 1));
      i = i + 1;
      if (outside_image(i))
        break;
      end
    end

    plot(velo_img(i-1,1),velo_img(i-1,2),'x','LineWidth',4, 'Color', 'r');

    if (abs(current_colour - starting_colour) < colour_thresh)
      % TODO: plot pixels between x_start and x_end, not velo_img_start and velo_img_end
      % add points between idx_2 and i to mask
      for j = idx_2:i-1
        if (outside_image(img, velo_img, j))
          break;
        end
        plot(velo_img(j, 1), velo_img(j, 2), 'x','LineWidth',4, 'MarkerSize', 1, 'Color', 'm');
        % bg_mask(round(velo_img(j, 2)), round(velo_img(j, 1)), 1:3) = 1;
        bg_points(round(velo_img(j, 2)), round(velo_img(j, 1)), 1:3) = img(round(velo_img(j, 2)), round(velo_img(j, 1)), 1:3);
      end
    end
  end
end