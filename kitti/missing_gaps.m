function fg_points = missing_gaps(img, velo_img, velo, opt_gap_thresh)
    % lekker for frame 0
    % useful when not using height thresholding

% finds and plots 'gaps' where the are no velodyne points
% make sure you are not thresholding for height when using
% 
% Parameters:
% img               | image to plot on
% velo_img          | LiDAR point cloud projected onto img
% velo              | LiDAR point cloud
% opt_grad_thresh   | optional gap threshold (defaults to 20)

if nargin > 3
    gap_thresh = opt_gap_thresh;
else
    gap_thresh = 20;
end

% provide a threshold for large gaps (i.e. stretches of road that shouldn't be masked)
big_gap_thresh = 200;

fg_points = uint8(zeros(size(img)));
for i = 2:size(velo_img, 1)
  % ignore points high up
  if (velo(i,3) > 0.2)
    continue
  end

  prev_x_pixel_pos = velo_img(i-1, 1);
  x_pixel_pos = velo_img(i, 1);

  % ignore big gaps at the back
  if (abs(x_pixel_pos-prev_x_pixel_pos) > big_gap_thresh) 
      continue;
  end

  if (abs(x_pixel_pos-prev_x_pixel_pos) > gap_thresh)
    for j = x_pixel_pos+1:prev_x_pixel_pos-1
      if (j > size(img, 2))
        break
      end
      plot(j, velo_img(i,2), 'o','LineWidth',4, 'MarkerSize', 1, 'Color', 'y');
      % fg_mask(round(velo_img(i, 2)), round(j), 1:3) = 1;
      fg_points(round(velo_img(i, 2)), round(j), 1:3) = img(round(velo_img(i, 2)), round(j), 1:3);
    end
  end
end