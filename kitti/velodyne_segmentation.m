% base_dir  = '/home/josh/Documents/UCT/Thesis/Datasets/2011_09_26/2011_09_26_drive_0093_sync';
% base_dir  = '/home/josh/Documents/UCT/Thesis/Datasets/2011_09_26/2011_09_26_drive_0009_sync';
% base_dir  = '/home/josh/Documents/UCT/Thesis/Datasets/2011_09_26/2011_09_26_drive_0013_sync';
base_dir  = '/home/josh/Documents/UCT/Thesis/Datasets/2011_09_30/2011_09_30_drive_0027_sync';

% keep these global to access from read_velo.m
global base_dir;
global img;

base_dir = '/home/josh/Documents/UCT/Thesis/Datasets/2011_09_30/2011_09_30_drive_0027_sync';

calib_dir = '/home/josh/Documents/UCT/Thesis/Datasets/2011_09_30/';
sdk_dir = '/home/josh/Documents/UCT/Thesis/Datasets/KITTI_devkit/matlab/';
odo_dir = '/home/josh/Documents/UCT/Thesis/Datasets/KITTI_odometry_devkit/dataset/poses/';
addpath(sdk_dir);

cam       = 2; % 0-based index
% frame = 392 for drive 93
% frame = 329 for drive 09
% frame = 42 for drive 13
frame     = 397; % 0-based index
forward_frames = 3;
backward_frames = 3;
odo_sequence = 7;

% load calibration
calib = loadCalibrationCamToCam(fullfile(calib_dir,'calib_cam_to_cam.txt'));
Tr_velo_to_cam = loadCalibrationRigid(fullfile(calib_dir,'calib_velo_to_cam.txt'));
odo_calib = load_odometry(fullfile([odo_dir, sprintf('0%d.txt', odo_sequence)]));
if isempty(odo_calib)
  disp('Failed to read odometry data');
end

% compute projection matrix velodyne->image plane
R_cam_to_rect = eye(4);
R_cam_to_rect(1:3,1:3) = calib.R_rect{1};
% P3 (left colour cam) * cam->cam calibration * velo->cam calibration
P_velo_to_img = calib.P_rect{cam+1}*R_cam_to_rect*Tr_velo_to_cam;

% load and display image
img = imread(sprintf('%s/image_%02d/data/%010d.png',base_dir,cam,frame));
lab_img = rgb2lab(img);
fig = figure('Position',[20 100 size(img,2) size(img,1)]); axes('Position',[0 0 1 1]);
imshow(img); hold on;
% axis on; grid on;

[base_velo, base_velo_img, bg_velo] = read_velo(frame, P_velo_to_img);

bg_velo_img = project(bg_velo(:,1:3),P_velo_to_img);

% remove points from bg_img outside img
i = 1;
while (i <= size(bg_velo_img, 1))
  if outside_image(img, bg_velo_img, i)
    bg_velo_img(i,:) = [];
    bg_velo(i, :) = [];
  else
    i = i + 1;
  end
end

multi_velo_img = base_velo_img;
multi_velo = base_velo;

% get velo points from multiple frames
for f = frame-backward_frames:frame+forward_frames
  if (f == frame)
    continue
  end

  % determine transform from forward/backward frames to base frame
  odo_diff = odo_calib{frame+1}\odo_calib{f+1};

  % build transform matrix
  P_velo_to_img = calib.P_rect{cam + 1} * R_cam_to_rect * odo_diff * Tr_velo_to_cam;

  % load velodyne points
  [velo, velo_img] = read_velo(f, P_velo_to_img);

  % find nearest y,z neighbour in base frame (x is depth(forward))
  nearest_yz = knnsearch(base_velo(:,2:3), velo(:,2:3));
  % % set depth to be that of nearest neighbour
  velo(:,1) = base_velo(nearest_yz, 1);

  multi_velo_img = [multi_velo_img; velo_img];
  multi_velo = [multi_velo; velo];
end

% plot points
colours = jet;
col_idx = round(64*5./velo(:,1));

rgb_matrix = zeros(size(velo_img, 1), 3);
bg_rgb_matrix = zeros(size(bg_velo_img, 1), 3);
ab_matrix = zeros(size(velo_img, 1), 2);
rows = round(velo_img(:,2));
cols = round(velo_img(:,1));

% rgb_matrix(:, 1:3) = img(rows, cols, 1:3);


for i=1:size(velo_img,1)
  % colours = cols = 64 x 3
  % 5 main colours (more and it breaks?)
  % min(velo(:,1)) == 5
  plot(velo_img(i,1),velo_img(i,2),'o','LineWidth',4,'MarkerSize',1,'Color',colours(col_idx(i),:));
  % mask(round(velo_img(i, 2)), round(velo_img(i, 1))) = 1;
  rgb_matrix(i, 1:3) = img(rows(i), cols(i), 1:3);
  ab_matrix(i, 1:2) = lab_img(rows(i), cols(i), 2:3);
end

bg_col_idx = round(64*5./bg_velo(:,1));
% for i = 1:size(bg_velo_img, 1)
%   plot(bg_velo_img(i, 1), bg_img(i, 2), 'x', 'LineWidth', 4, 'MarkerSize', 1, 'Color', colours(bg_col_idx(i),:));
% end

bg_rows = round(bg_velo_img(:, 2));
bg_cols = round(bg_velo_img(:, 1));
for i = 1:size(bg_velo_img, 1)
  bg_rgb_matrix(i, 1:3) = img(bg_rows(i), bg_cols(i), 1:3);
end

% stores co-ordinates and rgb values of each pixel
img_matrix = zeros(size(img, 1)*size(img, 2), 5);
img_matrix_lab = zeros(size(img, 1)*size(img, 2), 4);

i = 1;
for col = 1:size(img, 2)
  for row = 1:size(img, 1)
    % [row, col, img(row, col, 1:3)]?
    % this is fucking dumb
    % _everything_ needs to be 'double', because uint8 clips at 255
    % I realized this and simply converted row and col
    % but the colours also need to be converted
    img_matrix(i, :) = [double(row), double(col), double(img(row, col, 1)), double(img(row, col, 2)), double(img(row, col, 3))];
    img_matrix_lab(i, :) = [double(row), double(col), double(lab_img(row, col, 1)), double(lab_img(row, col, 2))];
    i = i + 1;
  end
end

% matrix to store variables for clustering based on Euclidean distance
% format:
% velo_img_x, velo_img_y, depth, r, g, b
pointcloud_matrix = [velo_img col_idx rgb_matrix];
pointcloud_matrix_lab = [velo_img col_idx ab_matrix];

bg_pointcloud_matrix = [bg_velo_img bg_col_idx bg_rgb_matrix];

% this could be made higher, i.e. over-cluster
% and then merge similar clusters together
% where similarity is based on depth, colour, position
num_clusters = 15;
bg_num_clusters = 5;
 
% weights = [1; 1; 50; 0.5; 0.5; 0.5];
weights = [1; 1; 100; 0; 0; 0];
rgb_weights = [0; 0; 100; 10; 10; 10];
weights_lab = [10; 5; 100; 1; 1];
weighted_euc = @(XI, XJ, W) sqrt(bsxfun(@minus, XI, XJ).^2 * W);

Y = pdist(double(pointcloud_matrix), @(XI, XJ) weighted_euc(XI, XJ, weights));
Z = linkage(Y);
T = cluster(Z, 'maxclust', num_clusters);

bg_Y = pdist(double(bg_pointcloud_matrix), @(XI, XJ) weighted_euc(XI, XJ, weights));
bg_Z = linkage(bg_Y);
bg_T = cluster(bg_Z, 'maxclust', bg_num_clusters);
% T = cluster(Z, 'cutoff', 1.5, 'Depth', 20);

bg_polygons = [];
bg_cluster_points = [];
bg_idx = 1;
for i = 1:bg_num_clusters
  bg_cluster_id = find(bg_T == i);
  bg_cluster_matrix = [bg_pointcloud_matrix(bg_cluster_id, 2), bg_pointcloud_matrix(bg_cluster_id, 1), bg_pointcloud_matrix(bg_cluster_id, 3:6)];
  % plot(bg_pointcloud_matrix(bg_cluster_id, 1), bg_pointcloud_matrix(bg_cluster_id, 2), 'color', clust_col);
  if (numel(bg_cluster_id) > 30)
    K = convhull(bg_pointcloud_matrix(bg_cluster_id, 1), bg_pointcloud_matrix(bg_cluster_id, 2));
    pgon = polyshape(bg_cluster_matrix(K, 2), bg_cluster_matrix(K,1));
    bg_polygons = [bg_polygons; pgon];

    index = bg_idx.*ones(numel(bg_cluster_id), 1);
    bg_cluster_points = [bg_cluster_points; index, bg_pointcloud_matrix(bg_cluster_id,2), bg_pointcloud_matrix(bg_cluster_id,1), bg_pointcloud_matrix(bg_cluster_id,3:6)];
    bg_idx = bg_idx + 1;
  end
end
% plot(bg_polygons);

% store polygons (convex hull shapes)
% figure(); imshow(img); hold on;
polygons = [];

% store the points inside each polygon
num_cluster_points = [];

% used to keep track of which cluster points are in which polygon
poly_idx = 1;
for i = 1:num_clusters
  found_bg_clust = false;
  cluster_id = find(T==i);
% mask = zeros(size(img));
% cluster_id = find(T==10);
  clust_col = rand(1,3);

  % store indeces and rgb values of each cluster pos
  cluster_matrix = [pointcloud_matrix(cluster_id, 2), pointcloud_matrix(cluster_id, 1), pointcloud_matrix(cluster_id, 3:6)];
  cluster_matrix_lab = [pointcloud_matrix_lab(cluster_id, 2), pointcloud_matrix_lab(cluster_id, 1), pointcloud_matrix_lab(cluster_id, 4:5)];

  if (numel(cluster_id) > 30)
    % TODO: remove 'cluster_matrix' and just use pointcloud_matrix
    % K = convhull(cluster_matrix(:,2), cluster_matrix(:,1));

    for j = 1:bg_num_clusters
      bg_clust_idx = find(bg_T == j);

      % calculate colour and positional differences between current fg_cluster
      % and all bg_clusters
      % if they're very close, assume current fg_cluster is actually part of background
      col_dist = hist_colour_dist(bg_pointcloud_matrix(bg_clust_idx, :), pointcloud_matrix(cluster_id, :));
      p_dist = pos_dist(bg_pointcloud_matrix(bg_clust_idx, :), pointcloud_matrix(cluster_id, :));
      if (col_dist < 0.4 && p_dist < 1e05)
        % disp('Similar clusters found');
        % TODO: add fg points to bg points
        col = rand(1,3);
        found_bg_clust = true;
        % plot(bg_pointcloud_matrix(bg_clust_idx, 1), bg_pointcloud_matrix(bg_clust_idx, 2), 'color', col);
        % plot(pointcloud_matrix(cluster_id, 1), pointcloud_matrix(cluster_id, 2), 'color', col);
        break;
      end
    end

    if (found_bg_clust)
      continue;
    end

    index = poly_idx.*ones(numel(cluster_id), 1);
    temp = [index, cluster_matrix(:,2), cluster_matrix(:,1), cluster_matrix(:, 3:6)];
    K = convhull(pointcloud_matrix(cluster_id, 1), pointcloud_matrix(cluster_id, 2));
    pgon = polyshape(cluster_matrix(K, 2), cluster_matrix(K,1));
    polygons = [polygons; pgon];

    num_cluster_points = [num_cluster_points; index, cluster_matrix(:,2), cluster_matrix(:,1), cluster_matrix(:,3:6)];
    poly_idx = poly_idx + 1;
    % num_points = numel(cluster_matrix(:,1))
    % if num_points > point threshold && distance (x,y,z) > something 
    % then point is background and remove from clusters
    % end

    % roi = poly2mask(cluster_matrix(K,2), cluster_matrix(K, 1), size(img,1), size(img,2));
    % BW = grabcut(img, L, roi);
    % figure();
    % imshow(BW);
    % plot(pgon);
    %plot(cluster_matrix(K, 2), cluster_matrix(K, 1), 'r');
  end
end

% no polygons found
% :( :( :(
if isempty(polygons)
  disp('No objects detected.');
  return
end

% remove lower triangular since symmetric
poly_intersects = triu(overlaps(polygons));
% remove diagonal elements
poly_intersects = triu(poly_intersects, 1) + tril(poly_intersects, -1);

% r, c gives polygon indeces for first and second polygon respectively
[r, c] = find(poly_intersects == 1);
for i = 1:size(r)
  % number of (velodyne) cluster points from first cluster in polygon intersection
  cluster_idx = find(num_cluster_points(:,1) == r(i));
  intersection = intersect(polygons(r(i)), polygons(c(i)));
  [in, on] = inpolygon(num_cluster_points(cluster_idx, 2), num_cluster_points(cluster_idx, 3), intersection.Vertices(:,1), intersection.Vertices(:,2));
  inon = in | on; % combine in and on

  % remove points from cluster that are not part of intersection
  cluster_idx( ~any(cluster_idx.*inon, 2), :) = [];

  % calculate 'depth' of points in intersection from first cluster
  average_depth = mean(num_cluster_points(cluster_idx, 4));

  % number of (velodyne) cluster points from second cluster in polygon intersection
  cluster_idx_2 = find(num_cluster_points(:,1) == c(i));
  [in_2, on_2] = inpolygon(num_cluster_points(cluster_idx_2, 2), num_cluster_points(cluster_idx_2, 3), intersection.Vertices(:,1), intersection.Vertices(:,2));
  inon_2 = in_2 | on_2; % combine in and on

  % remove points from cluster that are not part of intersection
  cluster_idx_2( ~any(cluster_idx_2.*inon_2, 2), :) = [];

  % calculate 'depth' of points in intersection from first cluster
  average_depth_2 = mean(num_cluster_points(cluster_idx_2, 4));

  % remove polygon with fewer cluster points in intersection
  % if (numel(inon) > numel(inon_2))
  %   polygons(c(i)) = subtract(polygons(c(i)), polygons(r(i)));
  % else
  %   polygons(r(i)) = subtract(polygons(r(i)), polygons(c(i)));
  % end

  % remove polygon further back (keep thing in front)
  if (average_depth > average_depth_2)
     polygons(c(i)) = subtract(polygons(c(i)), polygons(r(i)));
  else
     polygons(r(i)) = subtract(polygons(r(i)), polygons(c(i)));
  end
end

plot(polygons);


  % for j = 1:numel(cluster_id)
  %   pos = cluster_id(j);
  %   plot(pointcloud_matrix(pos, 1), pointcloud_matrix(pos, 2), 'x', 'color', clust_col);
  %   % mask(rows(pos), cols(pos), 1:3) = 1;
  %   % find min x co-ordinate
  % end

  % could add neighbouring points to cluster, then decrease distance metric and repeat, until no more points are added (i.e. keep decreasing distance)
  % can also use custom distance here to give greater weighting to colour/near neighbours
  % [Idx, D] = rangesearch(img_matrix, cluster_matrix, 5);
  % [Idx, D] = rangesearch(img_matrix, cluster_matrix, 3);

  % for neighbours = 1:numel(Idx)
  %   row = img_matrix(Idx{neighbours}, :);
  %   plot(row(:, 2), row(:, 1), 'x', 'color', clust_col);
  % end

%  
%  maskedRgbImage = bsxfun(@times, img, cast(mask,class(img)));
%
%  for i = 1:size(cols)
%    for j = 1:size(rows)
%      r = img(i, j, 1);
%      g = img(i, j, 2);
%      b = img(i, j, 3);
%
%      mask_r = maskedRgbImage(i, j, 1);
%      mask_g = maskedRgbImage(i, j, 2);
%      mask_b = maskedRgbImage(i, j, 3);
%
%    end
%  end
%
%
%  fg_mask = (zeros(size(img)));
%  bg_mask = (zeros(size(img)));
%
%  % Active contour not working well
%  % bw = activecontour(img, mask, 300);
%  % contour(bw, 'Color', 'm');
%
%  % figure();
%  % imshow(img); hold on; axis on; grid on;
%
% gradient_edges(img, velo_img, velo);
% missing_gaps(img, velo_img_copy, velo_copy);
%  % colour_difference(img, velo_img, velo);
%  % threshold_by_depth(img, velo_img, velo);

function colour_dist = hist_colour_dist(bg_clust, fg_clust)
  % build colour histograms for bg_clust and fg_clust
  % compute differences btw. histograms
  % returns overall distance
  % smaller = more similar

  % get r, g, b
  bg_r = bg_clust(:, 4);
  bg_g = bg_clust(:, 5);
  bg_b = bg_clust(:, 6);

  r = fg_clust(:, 4);
  g = fg_clust(:, 5);
  b = fg_clust(:, 6);

  % build histograms
  bg_r = imhist(bg_r./255, 16);
  bg_g = imhist(bg_g./255, 16);
  bg_b = imhist(bg_b./255, 16);

  r = imhist(r./255, 16);
  g = imhist(g./255, 16);
  b = imhist(b./255, 16);

  % normalize histograms
  bg_r = bg_r./numel(bg_clust);
  bg_g = bg_g./numel(bg_clust);
  bg_b = bg_b./numel(bg_clust);

  r = r./numel(fg_clust);
  g = g./numel(fg_clust);
  b = b./numel(fg_clust);

  % compute histogram differences
  res_r = sum(abs(bg_r - r));
  res_g = sum(abs(bg_g - g));
  res_b = sum(abs(bg_b - b));

  % sum resulting differences
  colour_dist = res_r + res_g + res_b;
end

function dist = pos_dist(bg_clust, fg_clust)
  % computes distance between average x,y of fg and bg clusters
  bg_x = mean(bg_clust(:,1));
  bg_y = mean(bg_clust(:,2));

  x = mean(fg_clust(:,1));
  y = mean(fg_clust(:,2));

  x_dist = abs(x-bg_x); 
  y_dist = abs(y-bg_y);

  % ignore sqrt for computational efficiency
  dist = x_dist^2+y_dist^2;

end

function [success, intra_clusts] = multi_intra_clust(clust)
  % determines if there are multiple clusters within a single cluster
  % uses colour as determining factor
  %
  % input:
  % clust - pointcloud_matrix rows (x, y, depth, r, g, b)
  % ------------------------------------------------
  % output:
  % intra_clusts is a cell array - each cell corresponds to an intra cluster


  % things to think about:
  % could convert to grayscale, use Otsu's thresholding
  % could try grouping nearest neighbours
  % could try re-running heirarchical clustering based purely on r,g,b and x,y
  % could try building histograms and finding distinct valleys
  % could try segmenting via Lab with a set number of clusters

  r = clust(:, 4);
  g = clust(:, 5);
  b = clust(:, 6);

  r = imhist(r./255, 16);
  g = imhist(g./255, 16);
  b = imhist(b./255, 16);

  r = r./numel(fg_clust);
  g = g./numel(fg_clust);
  b = b./numel(fg_clust);

end