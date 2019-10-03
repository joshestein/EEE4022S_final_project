% keep these global to access from read_velo.m
global base_dir;
global img;

% base_dir  = '/home/josh/Documents/UCT/Thesis/Datasets/2011_09_26/2011_09_26_drive_0009_sync'; % city
% base_dir  = '/home/josh/Documents/UCT/Thesis/Datasets/2011_09_26/2011_09_26_drive_0013_sync'; % city
% base_dir  = '/home/josh/Documents/UCT/Thesis/Datasets/2011_09_26/2011_09_26_drive_0048_sync'; % city
% base_dir  = '/home/josh/Documents/UCT/Thesis/Datasets/2011_09_26/2011_09_26_drive_0093_sync'; % city
% base_dir  = '/home/josh/Documents/UCT/Thesis/Datasets/2011_09_28/2011_09_28_drive_0034_sync'; % campus
% base_dir  = '/home/josh/Documents/UCT/Thesis/Datasets/2011_09_28/2011_09_28_drive_0038_sync'; % campus
% base_dir = '/home/josh/Documents/UCT/Thesis/Datasets/2011_09_30/2011_09_30_drive_0020_sync';  % residential
% base_dir = '/home/josh/Documents/UCT/Thesis/Datasets/2011_09_30/2011_09_30_drive_0027_sync';  % residential
% base_dir = '/home/josh/Documents/UCT/Thesis/Datasets/2011_09_30/2011_09_30_drive_0034_sync';  % residential
base_dir = '/home/josh/Documents/UCT/Thesis/Datasets/2011_10_03/2011_10_03_drive_0027_sync'; % road
% base_dir = '/home/josh/Documents/UCT/Thesis/Datasets/2011_10_03/2011_10_03_drive_0042_sync'; % road

% calib_dir = '/home/josh/Documents/UCT/Thesis/Datasets/2011_09_26/';
% calib_dir = '/home/josh/Documents/UCT/Thesis/Datasets/2011_09_28/';
% calib_dir = '/home/josh/Documents/UCT/Thesis/Datasets/2011_09_30/';
calib_dir = '/home/josh/Documents/UCT/Thesis/Datasets/2011_10_03/';

save_dir = "full_run/drive_27/no_merge/";
bb_dir = "full_run/10_03_drive_27/bb/";

sdk_dir = '/home/josh/Documents/UCT/Thesis/Datasets/KITTI_devkit/matlab/';
odo_dir = '/home/josh/Documents/UCT/Thesis/Datasets/KITTI_odometry_devkit/dataset/poses/';
addpath(sdk_dir);

cam       = 2; % 0-based index
frame     = 71; % 0-based index
forward_frames = 0;
backward_frames = 0;
num_frames = 1;   % incremented when reading velo data, in case frames extend pass file poundaries.
odo_sequence = 7; % ground-truth odometry poses for this sequence

% Odometry sequences:
% 00: 2011_10_03_drive_0027 000000 004540
% 01: 2011_10_03_drive_0042 000000 001100
% 02: 2011_10_03_drive_0034 000000 004660
% 03: 2011_09_26_drive_0067 000000 000800
% 04: 2011_09_30_drive_0016 000000 000270
% 05: 2011_09_30_drive_0018 000000 002760
% 06: 2011_09_30_drive_0020 000000 001100
% 07: 2011_09_30_drive_0027 000000 001100
% 08: 2011_09_30_drive_0028 001100 005170
% 09: 2011_09_30_drive_0033 000000 001590
% 10: 2011_09_30_drive_0034 000000 001200

num_files = dir(sprintf('%s/image_%02d/data/', base_dir, cam));
bb_files = dir(bb_dir);
% subtract '.' and '..'
num_files = size(num_files, 1) - 2;

file_id = fopen(sprintf('%stiming.txt', bb_dir), 'w');
fprintf(file_id, 'Time,Polygons\n');

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

for file = 1:size(bb_files, 1)
  close all;
  frame = bb_files(file).name;
  if (strcmp(frame, '.') || strcmp(frame, '..') || strcmp(frame, '.mat'))
    continue;
  end
  frame = str2double(frame(1:end-4));
  disp(frame)

  % load and display image
  img = imread(sprintf('%s/image_%02d/data/%010d.png',base_dir,cam,frame));
  fig = figure('Visible', 'off');
  imshow(img); hold on;
  % axis on; grid on;

  [base_velo, base_velo_img] = bb_read_velo(frame, P_velo_to_img);

  multi_velo_img = base_velo_img;
  multi_velo = base_velo;

  t_start = tic;

  % get velo points from multiple frames
  for f = frame-backward_frames:frame+forward_frames
    if (f < 1)
      continue;
    end

    if (f > num_files - 1)
      break;
    end

    if (f == frame)
      continue;
    end

    % determine transform from forward/backward frames to base frame
    odo_diff = odo_calib{frame+1}\odo_calib{f+1};

    % build transform matrix
    P_velo_to_img = calib.P_rect{cam + 1} * R_cam_to_rect * odo_diff * Tr_velo_to_cam;

    % load velodyne points
    [velo, velo_img] = bb_read_velo(f, P_velo_to_img);

    multi_velo_img = [multi_velo_img; velo_img];
    multi_velo = [multi_velo; velo];
    
    num_frames = num_frames + 1;
  end

  % colours = jet;
  % col_idx = round(64*min(multi_velo_img(:,3))./multi_velo_img(:,3));

  % rgb_matrix = zeros(size(multi_velo_img, 1), 3);
  % rows = round(multi_velo_img(:,2));
  % cols = round(multi_velo_img(:,1));
  % % rgb_matrix(:, 1:3) = img(rows, cols, 1:3);
  % 
  % for i=1:size(multi_velo_img,1)
  %   try
  %     % plot(multi_velo_img(i,1),multi_velo_img(i,2),'o','LineWidth',4,'MarkerSize',1,'Color',colours(col_idx(i),:));
  %   catch
  %     continue;
  %   end
  %   rgb_matrix(i, 1:3) = img(rows(i), cols(i), 1:3);
  % end

  polygons = [];
  [rows, cols, channels] = size(img);
  bb_rect = load(sprintf('%s%d.mat', bb_dir, frame));
  bb_rect = bb_rect.bb;

  for rect_id = 1:size(bb_rect, 1)
    if (all(bb_rect(rect_id)) == 0)
      continue;
    end
    y_min = bb_rect(rect_id, 1) * rows;
    x_min = bb_rect(rect_id, 2) * cols;
    y_max = bb_rect(rect_id, 3) * rows;
    x_max = bb_rect(rect_id, 4) * cols;

    important_idx = (multi_velo_img(:,1) > x_min) & (multi_velo_img(:,1) < x_max) & (multi_velo_img(:,2) > y_min) & (multi_velo_img(:,2) < y_max);
    important_velo = multi_velo_img(important_idx, :);

    % distant points assume TF does better and don't cluster
    if (mean(important_velo(:, 3)) > 30)
      pgon = polyshape([x_min, x_max, x_max, x_min], [y_min, y_min, y_max, y_max]);
    else
      weights = [1; 1; 1000]; 
      weighted_euc = @(XI, XJ, W) sqrt(bsxfun(@minus, XI, XJ).^2 * W);

      Y = pdist(double(important_velo), @(XI, XJ) weighted_euc(XI, XJ, weights));
      Z = linkage(Y);
      T = cluster(Z, 'maxclust', 5);

      % find cluster with max points
      max_points = 0;
      max_idx = 1;
      for i = 1:5
        clust_id = T == i;
        if (nnz(clust_id) > max_points)
          max_points = nnz(clust_id);
          max_idx = i;
        end
      end

      clust_id = T == max_idx;
      % plot(important_velo(clust_id, 1), important_velo(clust_id, 2), 'x', 'Color', 'r');
      K = convhull(important_velo(clust_id, 1), important_velo(clust_id, 2));
      cluster_matrix = [important_velo(clust_id, 1), important_velo(clust_id, 2)];

      pgon = polyshape(cluster_matrix(K, 1), cluster_matrix(K,2));
    end
    polygons = [polygons; pgon];

  end

  t_end = toc(t_start);
  fprintf(file_id, '%f, %d\n', t_end, size(polygons, 1));
  plot(polygons)

  F = getframe(gca);
  imwrite(F.cdata, sprintf('%s%d.png', bb_dir, frame));

  mask = zeros(size(img, 1), size(img,2));
  for i = 1:size(polygons)
    curr_poly_mask = poly2mask(polygons(i).Vertices(:,1), polygons(i).Vertices(:,2), size(img,1), size(img, 2));
    mask = mask + curr_poly_mask;
  end
  save(sprintf("%s%d_mask.mat", bb_dir, frame), 'mask');
end

fclose(file_id);

return;

% matrix to store variables for clustering based on Euclidean distance
% format:
% velo_img_x, velo_img_y, depth, r, g, b
pointcloud_matrix = [multi_velo_img rgb_matrix];

% this could be made higher, i.e. over-cluster
% and then merge similar clusters together
% where similarity is based on depth, colour, position
num_clusters = 10;
 
weights = [1.5; 1.5; 1000; 0; 0; 0]; 
weighted_euc = @(XI, XJ, W) sqrt(bsxfun(@minus, XI, XJ).^2 * W);

Y = pdist(double(pointcloud_matrix), @(XI, XJ) weighted_euc(XI, XJ, weights));
Z = linkage(Y);
T = cluster(Z, 'maxclust', num_clusters);

% store polygons (convex hull shapes)
% figure(); imshow(img); hold on;
bg_polygons = [];

% store the points inside each polygon
% format: cluster_number, y, x, depth, r, g, b
% cannot remember why I switched x and y
num_cluster_points = [];

% used to keep track of which cluster points are in which polygon
poly_idx = 1;

for i = 1:num_clusters
  found_bg_clust = false;
  cluster_id = find(T==i);
  numel(cluster_id);
% mask = zeros(size(img));
% cluster_id = find(T==10);
  clust_col = rand(1,3);

  % store indeces and rgb values of each cluster pos
  cluster_matrix = [pointcloud_matrix(cluster_id, 2), pointcloud_matrix(cluster_id, 1), pointcloud_matrix(cluster_id, 3:6)];
  % cluster_matrix_lab = [pointcloud_matrix_lab(cluster_id, 2), pointcloud_matrix_lab(cluster_id, 1), pointcloud_matrix_lab(cluster_id, 4:5)];

  % plot(cluster_matrix(:,2), cluster_matrix(:,1), 'x', 'color', clust_col);

  if (numel(cluster_id) > 30)
    % TODO: remove 'cluster_matrix' and just use pointcloud_matrix
    % K = convhull(cluster_matrix(:,2), cluster_matrix(:,1));

    index = poly_idx.*ones(numel(cluster_id), 1);
    K = convhull(pointcloud_matrix(cluster_id, 1), pointcloud_matrix(cluster_id, 2));
    pgon = polyshape(cluster_matrix(K, 2), cluster_matrix(K,1));
    polygons = [polygons; pgon];

    % num_cluster_points = [num_cluster_points; index, cluster_matrix(:,2), cluster_matrix(:,1), cluster_matrix(:,3:6)];
    num_cluster_points = [num_cluster_points; index, pointcloud_matrix(cluster_id,:)];
    poly_idx = poly_idx + 1;

  end
end

% no polygons found
% :( :( :(
if isempty(polygons)
  disp('No objects detected.');
  return
end

% combine similar polygons (that are stacked directly above one another)
% TODO: perhaps switch to using activecontour? Or finding _all_ similar pixels (i.e. above original bg pixels)
m = size(polygons, 1);
i = 1;
while (i < m)
  clust_1 = (num_cluster_points(:,1) == i);
  j = i + 1;
  while (j < m)
    % TODO: more than one fusion is breaking this
    clust_2 = (num_cluster_points(:,1) == j);
    col_dist = hist_colour_dist(num_cluster_points(clust_1, 2:7), num_cluster_points(clust_2, 2:7));
    p_dist = pos_dist(num_cluster_points(clust_1, 2:7), num_cluster_points(clust_2, 2:7));
    if (col_dist < 0.3 && p_dist < 4e03)
      % set clust_2 to be part of clust_1
      num_cluster_points(clust_2, 1) = i;
      new_clust = (num_cluster_points(:,1) == i);
      
      cluster_matrix = num_cluster_points(new_clust, :);

      K = convhull(num_cluster_points(new_clust, 2), num_cluster_points(new_clust, 3));
      pgon = polyshape(cluster_matrix(K, 2), cluster_matrix(K,3));

      polygons(i) = pgon;
      polygons(j) = [];
      m = m - 1;
    end
    j = j + 1;
  end
  i = i + 1;
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

  % calculate 'depth' of points in intersection from second cluster
  average_depth_2 = mean(num_cluster_points(cluster_idx_2, 4));

  % remove polygon further back (keep thing in front)
  if (average_depth > average_depth_2)
     polygons(r(i)) = subtract(polygons(r(i)), polygons(c(i)));
  else
     polygons(c(i)) = subtract(polygons(c(i)), polygons(r(i)));
  end
end

plot(polygons);

mask = zeros(size(img, 1), size(img,2));
for i = 1:size(polygons)
  curr_poly_mask = poly2mask(polygons(i).Vertices(:,1), polygons(i).Vertices(:,2), size(img,1), size(img, 2));
  mask = mask + curr_poly_mask;
end

% save(sprintf("%s%d_mask.mat", save_dir, frame), 'mask');
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

