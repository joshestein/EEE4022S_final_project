% NOTE:
% currently, reading velo and transforming to velo_img, but storing both.
% for efficiency can just use velo_img.
% Keeping both for security for now.

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
base_dir = '/home/josh/Documents/UCT/Thesis/Datasets/2011_09_30/2011_09_30_drive_0027_sync';  % residential
% base_dir = '/home/josh/Documents/UCT/Thesis/Datasets/2011_09_30/2011_09_30_drive_0034_sync';  % residential
% base_dir = '/home/josh/Documents/UCT/Thesis/Datasets/2011_10_03/2011_10_03_drive_0042_sync'; % road

% calib_dir = '/home/josh/Documents/UCT/Thesis/Datasets/2011_09_26/';
% calib_dir = '/home/josh/Documents/UCT/Thesis/Datasets/2011_09_28/';
calib_dir = '/home/josh/Documents/UCT/Thesis/Datasets/2011_09_30/';
% calib_dir = '/home/josh/Documents/UCT/Thesis/Datasets/2011_10_03/';


save_dir = "full_run/drive_27/no_merge/";

sdk_dir = '/home/josh/Documents/UCT/Thesis/Datasets/KITTI_devkit/matlab/';
odo_dir = '/home/josh/Documents/UCT/Thesis/Datasets/KITTI_odometry_devkit/dataset/poses/';
addpath(sdk_dir);

cam       = 2; % 0-based index
% frame = 392 for drive 93
% frame = 329 for drive 09
% frame = 42 for drive 13
% frame = 397 for drive 27
frame     = 50; % 0-based index
forward_frames = 0;
backward_frames = 0;
num_frames = 1;   % incremented when reading velo data, in case frames extend pass file poundaries.
% odo_sequence = 7; % ground-truth odometry poses for this sequence
odo_sequence = 1; % ground-truth odometry poses for this sequence

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
% subtract '.' and '..'
num_files = size(num_files, 1) - 2;

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
% lab_img = rgb2lab(img);
% figure;
% imshow(img); hold on;
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
  if (f < 1)
    continue;
  end

  if (f > num_files - 1)
    break;
  end

  if (f == frame)
    continue
  end

  % determine transform from forward/backward frames to base frame
  odo_diff = odo_calib{frame+1}\odo_calib{f+1};

  % build transform matrix
  P_velo_to_img = calib.P_rect{cam + 1} * R_cam_to_rect * odo_diff * Tr_velo_to_cam;

  % load velodyne points
  [velo, velo_img] = read_velo(f, P_velo_to_img);

  multi_velo_img = [multi_velo_img; velo_img];
  multi_velo = [multi_velo; velo];
  
  num_frames = num_frames + 1;
end

colours = jet;
col_idx = round(64*min(multi_velo_img(:,3))./multi_velo_img(:,3));

rgb_matrix = zeros(size(multi_velo_img, 1), 3);
bg_rgb_matrix = zeros(size(bg_velo_img, 1), 3);
% ab_matrix = zeros(size(multi_velo_img, 1), 2);
rows = round(multi_velo_img(:,2));
cols = round(multi_velo_img(:,1));
% rgb_matrix(:, 1:3) = img(rows, cols, 1:3);

for i=1:size(multi_velo_img,1)
  % colours = cols = 64 x 3
  % 5 main colours (more and it breaks?)
  % min(velo(:,1)) == 5
  % if (multi_velo_img(i ,3) > 26)
    % plot(multi_velo_img(i,1),multi_velo_img(i,2),'o','LineWidth',4,'MarkerSize',1,'Color',colours(col_idx(i),:));
  % end
  % mask(round(velo_img(i, 2)), round(velo_img(i, 1))) = 1;
  rgb_matrix(i, 1:3) = img(rows(i), cols(i), 1:3);
  % ab_matrix(i, 1:2) = lab_img(rows(i), cols(i), 2:3);
end

bg_col_idx = round(64*5./bg_velo(:,1));
% for i = 1:size(bg_velo_img, 1)
%   plot(bg_velo_img(i, 1), bg_velo_img(i, 2), 'x', 'LineWidth', 4, 'MarkerSize', 1, 'Color', colours(bg_col_idx(i),:));
% end

bg_rows = round(bg_velo_img(:, 2));
bg_cols = round(bg_velo_img(:, 1));
for i = 1:size(bg_velo_img, 1)
  bg_rgb_matrix(i, 1:3) = img(bg_rows(i), bg_cols(i), 1:3);
end

% keep distant points separate for different handling
dist_velo_idx = multi_velo_img(:,3) > 28;
dist_velo_img = multi_velo_img(dist_velo_idx, :);
dist_rgb_matrix = rgb_matrix(dist_velo_idx, :);
% remove distant points from multi_velo_img
multi_velo_img(dist_velo_idx, :) = [];
rgb_matrix(dist_velo_idx, :) = [];

% for i = 1:size(dist_velo_img, 1)
%   plot(dist_velo_img(i, 1), dist_velo_img(i, 2), 'x', 'MarkerSize', 10);
% end

% remove sparse points from dist and mult_velo matrices
[rows, cols, channels] = size(img);
for i = 1:16:cols 
  dist_idx = dist_velo_img(:,1) > i & dist_velo_img(:,1) < i+16;
  velo_idx = multi_velo_img(:,1) > i & multi_velo_img(:,1) < i+16;
  % if (dist_velo_img(:,1) > i && dist_velo_img(:,2) < i+10)
  points = nnz(dist_idx);
  sparse_velo_points = nnz(velo_idx);
  % if (sparse_velo_points < 15*num_frames)
  if (sparse_velo_points < 15)
    % plot(multi_velo_img(velo_idx, 1), multi_velo_img(velo_idx, 2), 'x', 'MarkerSize', 6);
    multi_velo_img(velo_idx, :) = [];
    rgb_matrix(velo_idx, :) = [];
  end
  if (points < 30)
    dist_velo_img(dist_idx, :) = [];
    dist_rgb_matrix(dist_idx, :) = [];
    % plot(dist_velo_img(dist_idx, 1), dist_velo_img(dist_idx, 2), 'x');
  end
end
% block_size = 16;
% for col = 1:block_size:size(img, 2)
%     % col + block_size = 1 + block points
%     % so we need to subtract one to ensure we are only searching for exactly block number of points.
%     % e.g block size = 16
%     % 1:(1+16) = searching 17 points
%     % So explicitely subtract 1.
%     col_end = col + block_size - 1;
%     for row = 1:block_size:size(img, 1)
%         row_end = row + block_size - 1;
%         num_points = nnz(dist_velo_img(:,1) > col & dist_velo_img(:,1) < col_end  & dist_velo_img(:,2) > row & dist_velo_img(:,2) < row_end);
%         if (num_points > 20)
%           rectangle('Position', [col row block_size block_size], 'EdgeColor', 'r');
%         end
%     end
% end

% stores co-ordinates and rgb values of each pixel
% used for a rangesearch later to find nearest pixels in image to velodyne points
% img_matrix = zeros(size(img, 1)*size(img, 2), 5);
% img_matrix_lab = zeros(size(img, 1)*size(img, 2), 4);
% 
% i = 1;
% for col = 1:size(img, 2)
%   for row = 1:size(img, 1)
%     % [row, col, img(row, col, 1:3)]?
%     % this is fucking dumb
%     % _everything_ needs to be 'double', because uint8 clips at 255
%     % I realized this and simply converted row and col
%     % but the colours also need to be converted
%     img_matrix(i, :) = [double(row), double(col), double(img(row, col, 1)), double(img(row, col, 2)), double(img(row, col, 3))];
%     img_matrix_lab(i, :) = [double(row), double(col), double(lab_img(row, col, 1)), double(lab_img(row, col, 2))];
%     i = i + 1;
%   end
% end

% matrix to store variables for clustering based on Euclidean distance
% format:
% velo_img_x, velo_img_y, depth, r, g, b
pointcloud_matrix = [multi_velo_img rgb_matrix];
% pointcloud_matrix_lab = [multi_velo_img ab_matrix];

bg_pointcloud_matrix = [bg_velo_img bg_rgb_matrix];
dist_pointcloud_matrix = [dist_velo_img dist_rgb_matrix];

% this could be made higher, i.e. over-cluster
% and then merge similar clusters together
% where similarity is based on depth, colour, position
num_clusters = 10;
bg_num_clusters = 15;
dist_num_clusters = 5;
 
weights = [1.5; 1.5; 1000; 0; 0; 0]; 
dist_weights = [1; 1; 0; 0; 0; 0]; 
bg_weights = [5; 1; 1; 5; 5; 5];
% weights_lab = [10; 5; 100; 1; 1];
weighted_euc = @(XI, XJ, W) sqrt(bsxfun(@minus, XI, XJ).^2 * W);

Y = pdist(double(pointcloud_matrix), @(XI, XJ) weighted_euc(XI, XJ, weights));
Z = linkage(Y);
T = cluster(Z, 'maxclust', num_clusters);

bg_Y = pdist(double(bg_pointcloud_matrix), @(XI, XJ) weighted_euc(XI, XJ, bg_weights));
if (isempty(bg_Y))
  bg_Z = [];
  bg_T = [];
else
  bg_Z = linkage(bg_Y);
  bg_T = cluster(bg_Z, 'maxclust', bg_num_clusters);
end

dist_cluster_points = [];
if (forward_frames ~= 0)
  dist_Y = pdist(double(dist_pointcloud_matrix), @(XI, XJ) weighted_euc(XI, XJ, dist_weights));
  dist_Z = linkage(dist_Y);
  dist_T = cluster(dist_Z, 'maxclust', dist_num_clusters);

  dist_idx = 1;
  for i = 1:dist_num_clusters
    clust_id = (dist_T == i);
    if (nnz(clust_id) > 30)
      index = dist_idx.*ones(nnz(clust_id), 1);
      dist_cluster_points = [dist_cluster_points; index, dist_pointcloud_matrix(clust_id, :)];
      dist_idx = dist_idx + 1;
      % plot(dist_pointcloud_matrix(clust_id, 1), dist_pointcloud_matrix(clust_id, 2), 'x');
    end
  end

  % combine distant clusters directly above one another
  if (~isempty(dist_cluster_points))
    for i = 1:dist_idx
      clust_1 = (dist_cluster_points(:,1) == i);
      x_1 = mean(dist_cluster_points(clust_1, 2));
      xrange_1 = range(dist_cluster_points(clust_1, 2));
      for j = i+1:dist_idx
        clust_2 = (dist_cluster_points(:,1) == j);
        x_2 = mean(dist_cluster_points(clust_2, 2));
        xrange_2 = range(dist_cluster_points(clust_2, 2));

        if (abs(x_2 - x_1) < 5 && abs(xrange_2 - xrange_1) < 20)
          dist_cluster_points(clust_2, 1) = i;
          % TODO: decrement subsequent dist_cluster_idx
        end
      end
    end
  end
end

% store polygons (convex hull shapes)
% figure(); imshow(img); hold on;
polygons = [];
bg_polygons = [];

% store the points inside each polygon
% format: cluster_number, y, x, depth, r, g, b
% cannot remember why I switched x and y
num_cluster_points = [];
bg_cluster_points = [];

% used to keep track of which cluster points are in which polygon
poly_idx = 1;
bg_idx = 1;

if (~isempty(bg_Y))
  for i = 1:bg_num_clusters
    bg_cluster_id = find(bg_T == i);
    bg_cluster_matrix = [bg_pointcloud_matrix(bg_cluster_id, 2), bg_pointcloud_matrix(bg_cluster_id, 1), bg_pointcloud_matrix(bg_cluster_id, 3:6)];
    % clust_col = rand(1,3);
    % plot(bg_pointcloud_matrix(bg_cluster_id, 1), bg_pointcloud_matrix(bg_cluster_id, 2), 'x', 'color', clust_col);
    if (numel(bg_cluster_id) > 30)
      K = convhull(bg_pointcloud_matrix(bg_cluster_id, 1), bg_pointcloud_matrix(bg_cluster_id, 2));
      pgon = polyshape(bg_cluster_matrix(K, 2), bg_cluster_matrix(K,1));
      bg_polygons = [bg_polygons; pgon];

      index = bg_idx.*ones(numel(bg_cluster_id), 1);
      bg_cluster_points = [bg_cluster_points; index, bg_pointcloud_matrix(bg_cluster_id,:)];
      % bg_cluster_points = [bg_cluster_points; index, bg_pointcloud_matrix(bg_cluster_id,2), bg_pointcloud_matrix(bg_cluster_id,1), bg_pointcloud_matrix(bg_cluster_id,3:6)];
      bg_idx = bg_idx + 1;
    elseif (numel(bg_cluster_id) > 10) % a small area of bg is likely something important, i.e. fg
      cluster_x_pos = mean(bg_pointcloud_matrix(bg_cluster_id), 1);
      
      % ignore if cluster is directly in middle of image
      % likely to be a gap in velodyne points
      if ((abs(cluster_x_pos) - size(img, 2)/2) < 20)
        continue;
      end

      % add polygon to fg_polygons
      K = convhull(bg_pointcloud_matrix(bg_cluster_id, 1), bg_pointcloud_matrix(bg_cluster_id, 2));
      pgon = polyshape(bg_cluster_matrix(K, 2), bg_cluster_matrix(K,1));
      polygons = [polygons; pgon];

      % add points to fg_cluster_points matrix
      index = poly_idx*ones(numel(bg_cluster_id), 1);
      num_cluster_points = [num_cluster_points; index, bg_pointcloud_matrix(bg_cluster_id,:)];
      poly_idx = poly_idx + 1;
    end
  end
end
% plot(bg_polygons);

% remove distant clusters that are similar to background
% TODO
% for i = 1:dist_num_clusters
%   dist_idx = find(dist_T == i);
%   for j = 1:size(bg_polygons, 1)
%     bg_clust_idx = (bg_cluster_points(:,1) == j);
% 
%     col_dist = hist_colour_dist(bg_cluster_points(bg_clust_idx, 2:7), dist_pointcloud_matrix(dist_idx, :));
%     if (col_dist < 0.4 )
%       found_bg_clust = true;
%       break;
%     end
%   end
% 
%   % if (found_bg_clust)
%   %   continue;
%   % end
%   col = rand(1,3);
%   % plot(dist_pointcloud_matrix(dist_idx, 1), dist_pointcloud_matrix(dist_idx, 2), 'x', 'color', col);
% end

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
    if (~isempty(bg_Y))

      for j = 1:bg_idx
        bg_clust_idx = (bg_cluster_points(:,1) == j);
        if (nnz(bg_clust_idx) < 15)
          continue;
        end

        % calculate colour and positional differences between current fg_cluster
        % and all bg_clusters
        % if they're very close, assume current fg_cluster is actually part of background
        col_dist = hist_colour_dist(bg_cluster_points(bg_clust_idx, 2:7), pointcloud_matrix(cluster_id, :));
        % col_dist = hist_colour_dist(bg_pointcloud_matrix(bg_clust_idx, :), pointcloud_matrix(cluster_id, :));
        p_dist = pos_dist(bg_cluster_points(bg_clust_idx, 2:7), pointcloud_matrix(cluster_id, :));
        % p_dist = pos_dist(bg_pointcloud_matrix(bg_clust_idx, :), pointcloud_matrix(cluster_id, :));
        if (col_dist < 0.4 && p_dist < 7e04)
          % disp('Similar clusters found');
          % TODO: add fg points to bg points
          % col = rand(1,3);
          found_bg_clust = true;
          % plot(bg_pointcloud_matrix(bg_clust_idx, 1), bg_pointcloud_matrix(bg_clust_idx, 2), 'x', 'color', col);
          % plot(pointcloud_matrix(cluster_id, 1), pointcloud_matrix(cluster_id, 2), 'o', 'color', col);
          break;
        end
      end

      if (found_bg_clust)
      % continue;
      end
    end

    index = poly_idx.*ones(numel(cluster_id), 1);
    K = convhull(pointcloud_matrix(cluster_id, 1), pointcloud_matrix(cluster_id, 2));
    pgon = polyshape(cluster_matrix(K, 2), cluster_matrix(K,1));
    polygons = [polygons; pgon];

    % num_cluster_points = [num_cluster_points; index, cluster_matrix(:,2), cluster_matrix(:,1), cluster_matrix(:,3:6)];
    num_cluster_points = [num_cluster_points; index, pointcloud_matrix(cluster_id,:)];
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

if (isempty(dist_cluster_points))
  % do nothing
else
  for i = 1:dist_idx
    cluster_id = (dist_cluster_points(:, 1) == i);
    if (nnz(cluster_id) == 0)
      continue;
    end
    index = poly_idx.*ones(nnz(cluster_id), 1);
    % dist_cluster_points(:,1) = poly_idx;
    num_cluster_points = [num_cluster_points; index dist_cluster_points(cluster_id, 2:7)];
    K = convhull(dist_cluster_points(cluster_id, 2), dist_cluster_points(cluster_id, 3));
    % only using this for the indexing
    % need to find a way to do dist_cluster_points(cluster_id)(K)
    % that is, K of dist_cluster_points(cluster_id)
    clust = dist_cluster_points(cluster_id, :);
    pgon = polyshape(clust(K, 2), clust(K, 3));
    polygons = [polygons; pgon];
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

% this removes adjacent polygons, keeping the innermost one
% the inner most is more likely to be a car, whereas the outermost is more likely to be a wall, bush, etc.
% working for frame 53, drive 27
% TODO: test on more images
i = 1;
while (i <= numel(r))
  % keep x and y positions to determine of objects are directly next to one another (not a significant change in y)
  try
    [x_1, y_1] = centroid(polygons(r(i)));
    [x_2, y_2] = centroid(polygons(c(i)));
  catch
    i = i + 1;
    continue;
  end
  
  if (isnan(x_1) || isnan(y_1))
    polygons(r(i)) = [];
    i = i + 1;
    continue;
  end

  if (isnan(x_2) || isnan(y_2))
    polygons(c(i)) = [];
    i = i + 1;
    continue;
  end

  max_x_1 = max(polygons(r(i)).Vertices(:,1));
  min_x_1 = min(polygons(r(i)).Vertices(:,1));

  max_x_2 = max(polygons(c(i)).Vertices(:,1));
  min_x_2 = min(polygons(c(i)).Vertices(:,1));

  % if polygon extends to border of image
  if (abs(max_x_1 - size(img,2)) < 1 || ...
      abs(max_x_2 - size(img,2)) < 1 || ...
      min_x_1 < 1 || ...
      min_x_2 < 1)

  % and significant same height
    if (abs(y_2 - y_1) < 50)
      if ((x_1 < size(img, 2)/2) && (x_2 < size(img, 2)/2)) % left half image plane
        if (x_1 < x_2)  % first poly is to left of second
          polygons(r(i)) = []; % remove first poly
        else
          polygons(c(i)) = [];
        end
      elseif ((x_1 > size(img, 2)/2) && (x_2 > size(img, 2)/2)) % right half image plane
        if (x_1 < x_2)
          polygons(c(i)) = [];
        else
          polygons(r(i)) = [];
        end
      end
    end
  end
  i = i + 1;
end

% plot(polygons);
polygons = interactive(img, polygons);

% mask = zeros(size(img, 1), size(img,2));
% for i = 1:size(polygons)
%   curr_poly_mask = poly2mask(polygons(i).Vertices(:,1), polygons(i).Vertices(:,2), size(img,1), size(img, 2));
%   mask = mask + curr_poly_mask;
% end
% 
% save(sprintf("%s%d_mask.mat", save_dir, frame), 'mask');

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

  % colour_weights = [1; 1; 0; 1; 1; 1];
  % % convert from RGB to Lab
  % clust(:,4:6) = rgb2lab(clust(:, 4:6));
  % Y = pdist(double(clust), @(XI, XJ) weighted_euc(XI, XJ, colour_weights));
  % Z = linkage(Y);
  % dendrogram(Z);
  % T = cluster(Z, 'maxclust', 4);

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