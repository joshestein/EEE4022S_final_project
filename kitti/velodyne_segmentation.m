base_dir  = '/home/josh/Documents/UCT/Thesis/Datasets/2011_09_26/2011_09_26_drive_0093_sync';
calib_dir = '/home/josh/Documents/UCT/Thesis/Datasets/2011_09_26/';
sdk_dir = '/home/josh/Documents/UCT/Thesis/Datasets/KITTI_devkit/matlab/';
path(path, '../../Datasets/KITTI_devkit/matlab/');
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

% remove points that have a height of ~ 0.5 m
% thresh of 1 is conservative
height_thresh = 1.2;
idx = velo(:,3) < -height_thresh;
velo(idx,:) = [];

% project to image plane (exclude luminance)
velo_img = project(velo(:,1:3),P_velo_to_img);

% remove points outside of image
% velo_img(velo_img(:,1) > size(img,2), :) = [];
% velo_img(velo_img(:,2) > size(img,1), :) = [];
% velo_img((velo_img < 0), :) = [];

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
grad_thresh = 2;
% colours = round(64*5/velo(:,1));
% fx = gradient(colours);
% if (abs(fx(i)) > grad_thresh)
% doesn't seem to be working...
mask = false(size(img, 1), size(img,2));
edge_points = false(size(img, 1), size(img,2));
draw_mask = false;

% figure();
% clf; imshow(img); hold on; axis on; grid on;
% grad_thresh = 2;
% % colours = round(64*5/velo(:,1));
% % fx = gradient(colours);
% % if (abs(fx(i)) > grad_thresh)
% % doesn't seem to be working...
% edge_points = false(size(img, 1), size(img,2));
% draw_mask = false;

% find edges based on gradient changes in velodyne
% make sure you are not thresholding for height when using
% for i = 2:size(velo_img, 1)
%   % ignore points outsize image
%   if (velo_img(i,1) > size(img,2) || round(velo_img(i, 1)) <= 0 || velo_img(i,2) > size(img,1) || round(velo_img(i, 2)) <= 0)
%     continue;
%   end
% 
%   old_x = round(64*5/velo(i-1,1));
%   x = round(64*5/velo(i,1));
% 
%   if (draw_mask)
%     mask(round(velo_img(i,2)), round(velo_img(i,1))) = true;
%   end
%   if (abs(x-old_x)/2 > grad_thresh)
%     % plot(velo_img(i,1),velo_img(i,2),'o','LineWidth',5,'Color',cols(colours(i),:));
%     plot(velo_img(i,1),velo_img(i,2),'o','LineWidth',4, 'MarkerSize', 1, 'Color', 'y');
%     plot(velo_img(i-1,1),velo_img(i-1,2),'o','LineWidth',4, 'MarkerSize', 1, 'Color', 'g');
%     edge_points(round(velo_img(i,2)), round(velo_img(i,1))) = true;
%     draw_mask = ~draw_mask;
%   end
% end
 
% find 'gaps' where the are no velodyne points
% make sure you are not thresholding for height when using
% gap_thresh = 20;
% fg_points = uint8(zeros(size(img)));
% for i = 2:size(velo_img, 1)
%   % ignore points outsize image
%   if (velo_img(i,1) > size(img,2) || round(velo_img(i, 1)) <= 0 || velo_img(i,2) > size(img,1) || round(velo_img(i, 2)) <= 0)
%     continue;
%   end
% 
%   prev_x_pixel_pos = velo_img(i-1, 1);
%   x_pixel_pos = velo_img(i, 1);
% 
%   % ignore points right in middle at end of beam
%   if ((x_pixel_pos >= 610) && (prev_x_pixel_pos <= 720))
%     continue;
%   end
% 
%   if (abs(x_pixel_pos-prev_x_pixel_pos) > gap_thresh)
%     for j = x_pixel_pos:prev_x_pixel_pos
%       if (j > size(img, 2))
%         break
%       end
%       plot(j, velo_img(i,2), 'o','LineWidth',4, 'MarkerSize', 1, 'Color', 'r');
%       fg_mask(round(velo_img(i, 2)), round(j), 1:3) = 1;
%       fg_points(round(velo_img(i, 2)), round(j), 1:3) = img(round(velo_img(i, 2)), round(j), 1:3);
%     end
%   end
% end

% plot(edge_points(:,1), edge_points(:,2));

% out_img = @(pos) (velo_img(pos,1) > size(img,2) || round(velo_img(pos, 1)) <= 0 || velo_img(pos,2) > size(img,1) || round(velo_img(pos, 2)) <= 0);
% clf; imshow(img); hold on; axis on;
% % 6 is bad
% % 8 is good
% % 9 is good
% % 10 is good
% % 11 is good
% % 12 starts losing detail in car a bit
% % 5 is good
% % 871 is just to right of far away car 
% % for loop cannot modify loop variable inside of itself
% % use a while instead
% % colour_thresh = 12;
% make sure you are not thresholding for height when using
% bg_points = uint8(zeros(size(img)));
% for colour_thresh = 8:11
%   i = 1;
%   while ( i < size(velo_img, 1))
%     % i = i + 1;
%     if (out_img(i))
%       i = i + 1;
%       continue;
%     end
%     starting_colour = round(64*5/velo(i,1));
%     current_colour = starting_colour;
%     plot(velo_img(i,1),velo_img(i,2),'x','LineWidth',4, 'Color', 'g');
%     while (abs(current_colour - starting_colour) < colour_thresh)
%       current_colour = round(64*5/velo(i, 1));
%       i = i + 1;
%       if (out_img(i))
%         break;
%       end
%     end
% 
%     idx_2 = i;
%     plot(velo_img(i,1),velo_img(i,2),'x','LineWidth',4, 'Color', 'y');
%     % i = i + 1;
%     starting_colour_2 = round(64*5/velo(i, 1));
% 
%     while (abs(current_colour - starting_colour_2) < colour_thresh)
%       current_colour = round(64*5/velo(i, 1));
%       i = i + 1;
%       if (out_img(i))
%         break;
%       end
%     end
% 
%     plot(velo_img(i-1,1),velo_img(i-1,2),'x','LineWidth',4, 'Color', 'r');
% 
%     if (abs(current_colour - starting_colour) < colour_thresh)
%       % TODO: plot pixels between x_start and x_end, not velo_img_start and velo_img_end
%       % add points between idx_2 and i to mask
%       for j = idx_2:i-1
%         if (out_img(j))
%           break;
%         end
%         plot(velo_img(j, 1), velo_img(j, 2), 'x','LineWidth',4, 'MarkerSize', 1, 'Color', 'm');
%         bg_mask(round(velo_img(j, 2)), round(velo_img(j, 1)), 1:3) = 1;
%         bg_points(round(velo_img(j, 2)), round(velo_img(j, 1)), 1:3) = img(round(velo_img(j, 2)), round(velo_img(j, 1)), 1:3);
%       end
%     end
%   end
% end