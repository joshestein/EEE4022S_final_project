gt_dir = "/home/josh/Documents/UCT/Thesis/Datasets/ground_truth_segmentation/ros_offline/KITTI_SEMANTIC/Validation_07/GT/";
base_dir = "/home/josh/Documents/UCT/Thesis/Datasets/ground_truth_segmentation/ros_offline/KITTI_SEMANTIC/Validation_07/RGB/";
frame_integration_1 = "/home/josh/Documents/UCT/Thesis/Code/kitti/full_run/drive_27/1_f_1_b/";
frame_integration_2 = "/home/josh/Documents/UCT/Thesis/Code/kitti/full_run/drive_27/2_b/";
segmentation_dir = "/home/josh/Documents/UCT/Thesis/Code/kitti/full_run/drive_27/no_integration/";
no_merge_dir = "/home/josh/Documents/UCT/Thesis/Code/kitti/full_run/drive_27/no_merge/";
interactive_dir = "/home/josh/Documents/UCT/Thesis/Code/kitti/full_run/drive_27/no_merge/interactive/";

% frame = 172;
files = dir(gt_dir);
num_files = size(files);

frame_int_jaccard_1 = zeros(size(num_files, 1)-2, 1);
frame_int_jaccard_2 = zeros(size(num_files, 1)-2, 1);
no_integration_jaccard = zeros(size(num_files, 1)-2, 1);
no_merge_jaccard = zeros(size(num_files, 1)-2, 1);
interactive_jaccard = zeros(size(num_files, 1)-2, 1);

for i = 1:num_files
    frame = files(i).name;
    if (strcmp(frame, '.') || strcmp(frame, '..'))
        continue;
    else
        frame = str2double(frame(1:end-4));
    end

    gt_img = imread(sprintf('%s%06d.png', gt_dir, frame));
    img = imread(sprintf('%s%06d.png', base_dir, frame));

    no_integration_mask = load(sprintf('%s%d_mask.mat', segmentation_dir, frame));
    frame_1_mask = load(sprintf('%s%d_mask.mat', frame_integration_1, frame));
    frame_2_mask = load(sprintf('%s%d_mask.mat', frame_integration_2, frame));
    no_merge_mask = load(sprintf('%s%d_mask.mat', no_merge_dir, frame));
    interactive_mask = load(sprintf('%s%d_interactive_mask.mat', interactive, frame));

    no_integration_mask = logical(no_integration_mask.no_integration_mask);
    frame_1_mask = logical(frame_1_mask.frame_1_mask);
    frame_2_mask = logical(frame_2_mask.frame_2_mask);
    no_merge_mask = logical(no_merge_mask.no_merge_mask);
    interactive_mask = logical(interactive_mask.interactive_mask);

    % figure;

    r = gt_img(:,:,1);
    g = gt_img(:,:,2);
    b = gt_img(:,:,3);

    car_mask = ((r == 64) & (g == 0) & (b == 128));
    person_mask = ((r == 63) & (g == 63) & (b == 0));

    full_mask = car_mask | person_mask;

    no_integration_jaccard(i) = jaccard(full_mask, mask);
    frame_int_jaccard_1(i) = jaccard(full_mask, frame_1_mask);
    frame_int_jaccard_2(i) = jaccard(full_mask, frame_2_mask);
    no_merge_jaccard(i) = jaccard(full_mask, no_merge_mask);
    interactive_jaccard(i) = jaccard(full_mask, interactive_mask);
end
% masked = bsxfun(@times, gt_img, cast(car_mask,class(gt_img)));

% binary_mask = (masked ~= 0);
% imshow(full_mask)

% imshow(car_mask);
