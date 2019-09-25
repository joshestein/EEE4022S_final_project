gt_dir = "/home/josh/Documents/UCT/Thesis/Datasets/ground_truth_segmentation/ros_offline/KITTI_SEMANTIC/Validation_07/GT/";
segmentation_dir = "/home/josh/Documents/UCT/Thesis/Code/kitti/full_run/drive_27/no_merge/interactive/";

frame = 172;
gt_img = imread(sprintf('%s%06d.png', gt_dir, frame));
img = imread(sprintf('%s%d.png', segmentation_dir, frame));
mask = load(sprintf('%s%d_interactive_mask.mat', segmentation_dir, frame));
mask = logical(mask.mask);

figure;

r = gt_img(:,:,1);
g = gt_img(:,:,2);
b = gt_img(:,:,3);

car_mask = ((r == 64) & (g == 0) & (b == 128));
person_mask = ((r == 63) & (g == 63) & (b == 0));

full_mask = car_mask | person_mask;
% masked = bsxfun(@times, gt_img, cast(car_mask,class(gt_img)));

% binary_mask = (masked ~= 0);
imshow(full_mask)

% imshow(car_mask);
