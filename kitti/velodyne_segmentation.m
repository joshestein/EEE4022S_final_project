base_dir  = '/home/josh/Documents/UCT/Thesis/Datasets/2011_09_26/2011_09_26_drive_0093_sync';
calib_dir = '/home/josh/Documents/UCT/Thesis/Datasets/2011_09_26/';
sdk_dir = '/home/josh/Documents/UCT/Thesis/Datasets/KITTI_devkit/matlab/';
addpath(sdk_dir);

cam       = 2; % 0-based index
frame     = 392; % 0-based index

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
velo = velo(1:5:end,:); % remove every 5th point for display speed
fclose(fid);

% remove all points behind image plane (approximation
idx = velo(:,1)<5;
velo(idx,:) = [];

% remove points that have a height of ~ 0.2 m
% thresh of 1 is conservative
% negative because velo is centered at 0 on each of it's axes
% that is, half the points are above z = 0 and half below
height_thresh = -1.2;
idx = velo(:,3) < height_thresh;
velo(idx,:) = [];

% project to image plane (exclude luminance)
velo_img = project(velo(:,1:3),P_velo_to_img);

% plot points
cols = jet;
for i=1:size(velo_img,1)
  % colours = cols = 64 x 3
  % 5 main colours (more and it breaks?)
  % min(velo(:,1)) == 5
  col_idx = round(64*5/velo(i,1));
  plot(velo_img(i,1),velo_img(i,2),'o','LineWidth',4,'MarkerSize',1,'Color',cols(col_idx,:));
end

fg_mask = (zeros(size(img)));
bg_mask = (zeros(size(img)));

figure();
imshow(img); hold on; axis on; grid on;

gradient_edges(img, velo_img, velo);
% missing_gaps(img, velo_img);
% colour_difference(img, velo_img, velo);
 