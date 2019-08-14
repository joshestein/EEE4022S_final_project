% base_dir  = '/home/josh/Documents/UCT/Thesis/Datasets/2011_09_26/2011_09_26_drive_0093_sync';
base_dir  = '/home/josh/Documents/UCT/Thesis/Datasets/2011_09_26/2011_09_26_drive_0009_sync';
calib_dir = '/home/josh/Documents/UCT/Thesis/Datasets/2011_09_26/';
sdk_dir = '/home/josh/Documents/UCT/Thesis/Datasets/KITTI_devkit/matlab/';
addpath(sdk_dir);

cam       = 2; % 0-based index
% frame = 392 for drive 93
frame     = 329; % 0-based index

% load calibration
calib = loadCalibrationCamToCam(fullfile(calib_dir,'calib_cam_to_cam.txt'));
Tr_velo_to_cam = loadCalibrationRigid(fullfile(calib_dir,'calib_velo_to_cam.txt'));

% compute projection matrix velodyne->image plane
R_cam_to_rect = eye(4);
R_cam_to_rect(1:3,1:3) = calib.R_rect{1};
P_velo_to_img = calib.P_rect{cam+1}*R_cam_to_rect*Tr_velo_to_cam;

% load and display image
img = imread(sprintf('%s/image_%02d/data/%010d.png',base_dir,cam,frame));
fig = figure('Position',[20 100 size(img,2) size(img,1)]); axes('Position',[0 0 1 1]);
imshow(img); hold on;
axis on; grid on;

% load velodyne points
fid = fopen(sprintf('%s/velodyne_points/data/%010d.bin',base_dir,frame),'rb');
velo = fread(fid,[4 inf],'single')';
% velo = velo(1:5:end,:); % remove every 5th point for display speed
fclose(fid);

% remove all points behind image plane (approximation
idx = velo(:,1)<5;
velo(idx,:) = [];

% remove points far in the distance
idx = velo(:,1) > 30;
velo(idx,:) = [];

% velo_copy is not thresholded for height
velo_copy = velo;

% remove points that have a height of ~ 0.2 m
% thresh of 1 is conservative
% negative because velo is centered at 0 on each of it's axes
% that is, half the points are above z = 0 and half below
height_thresh = -1.2;
idx = velo(:,3) < height_thresh;
velo(idx,:) = [];

% remove very things above a particular height ~ 2m
idx = velo(:,3) > 0.1;
velo(idx,:) = [];

% project to image plane (exclude luminance)
velo_img = project(velo(:,1:3),P_velo_to_img);
velo_img_copy = project(velo_copy(:,1:3),P_velo_to_img);

% remove all points outside of image
i = 1;
while (i < size(velo_img, 1))
  if outside_image(img, velo_img, i)
    velo_img(i,:) = [];
    velo(i, :) = [];
  else
    i = i + 1;
  end
end

% plot points
colours = jet;
col_idx = round(64*5./velo(:,1));
mask = (zeros(size(img, 1), size(img, 2)));

rgb_matrix = zeros(size(velo_img, 1), 3);
rows = round(velo_img(:,2));
cols = round(velo_img(:,1));

% rgb_matrix(:, 1:3) = img(rows, cols, 1:3);


for i=1:size(velo_img,1)
  % colours = cols = 64 x 3
  % 5 main colours (more and it breaks?)
  % min(velo(:,1)) == 5
  % plot(velo_img(i,1),velo_img(i,2),'o','LineWidth',4,'MarkerSize',1,'Color',colours(col_idx(i),:));
  % mask(round(velo_img(i, 2)), round(velo_img(i, 1))) = 1;
  rgb_matrix(i, 1:3) = img(rows(i), cols(i), 1:3);
end

% matrix to store variables for clustering based on Euclidean distance
% format:
% velo_img_x, velo_img_y, depth, r, g, b
dist_matrix = [velo_img col_idx rgb_matrix];

Y = pdist(dist_matrix, 'Euclidean');
Z = linkage(Y);
T = cluster(Z, 'maxclust', 20);

figure(); imshow(img); hold on;
new_colours = ['r'; 'g'; 'b'; 'w'; 'k'; 'y'; 'm'; 'c'; 'r'; 'm'];
for i = 1:10
  cluster_id = find(T==i);
  for j = 1:numel(cluster_id)
    pos = cluster_id(j);
    plot(dist_matrix(pos, 1), dist_matrix(pos, 2), 'x', 'color', new_colours(i));
   end
end

fg_mask = (zeros(size(img)));
bg_mask = (zeros(size(img)));

% Active contour not working well
% bw = activecontour(img, mask, 300);
% contour(bw, 'Color', 'm');

% figure();
% imshow(img); hold on; axis on; grid on;

% gradient_edges(img, velo_img, velo);
% missing_gaps(img, velo_img_copy, velo_copy);
% colour_difference(img, velo_img, velo);
% threshold_by_depth(img, velo_img, velo);
 