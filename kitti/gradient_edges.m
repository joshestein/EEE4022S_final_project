function edge_positions = gradient_edges(img, velo_img, velo, opt_grad_thresh)

% finds edges based on gradient differences in depth
% plots edges
% green is start of edge
% yello is end
% Parameters:
% img               | image to plot on
% velo_img          | LiDAR point cloud projected onto img
% velo              | LiDAR point cloud
% opt_grad_thresh   | optional gradient threshold (defaults to 2)
% % % % % % % % % % % % % % % % % % % % % % % % % % % %

% faster way to compute gradients?
% colours = round(64*5./velo(:,1));
% fx = gradient(colours);
% if (abs(fx(i)) > grad_thresh)
% doesn't seem to be working...

if nargin > 3
    grad_thresh = opt_grad_thresh;
else
    grad_thresh = 2;
end

% mask = false(size(img, 1), size(img,2));
edge_positions = false(size(img, 1), size(img,2));
draw_mask = false;

% figure();
% clf; imshow(img); hold on; axis on; grid on;
% colours = round(64*5/velo(:,1));
% fx = gradient(colours);
% if (abs(fx(i)) > grad_thresh)
% doesn't seem to be working...
edge_points = false(size(img, 1), size(img,2));
draw_mask = false;

% find edges based on gradient changes in velodyne
% make sure you are not thresholding for height when using
for i = 2:size(velo_img, 1)
  old_x = round(velo(i-1,1));
  x = round(velo(i,1));

  % if (draw_mask)
  %   mask(round(velo_img(i,2)), round(velo_img(i,1))) = true;
  % end
  if (abs(x-old_x)/2 > grad_thresh)
    % plot(velo_img(i,1),velo_img(i,2),'o','LineWidth',5,'Color',cols(colours(i),:));
    plot(velo_img(i,1),velo_img(i,2),'o','LineWidth',4, 'MarkerSize', 1, 'Color', 'y');
    plot(velo_img(i-1,1),velo_img(i-1,2),'o','LineWidth',4, 'MarkerSize', 1, 'Color', 'g');
    edge_points(round(velo_img(i,2)), round(velo_img(i,1))) = true;
    draw_mask = ~draw_mask;
  end
end