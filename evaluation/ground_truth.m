function jaccards = ground_truth()

% 2011_09_30_drive_27
% ros - 'Validation'
gt_dir = "/home/josh/Documents/UCT/Thesis/Datasets/ground_truth_segmentation/ros_offline/KITTI_SEMANTIC/Validation_07/GT/";
base_dir = "/home/josh/Documents/UCT/Thesis/Datasets/ground_truth_segmentation/ros_offline/KITTI_SEMANTIC/Validation_07/RGB/";
frame_integration_1 = "/home/josh/Documents/UCT/Thesis/Code/kitti/full_run/drive_27/1_f_1_b/no_merge/";
frame_integration_2 = "/home/josh/Documents/UCT/Thesis/Code/kitti/full_run/drive_27/2_b/no_merge/";
segmentation_dir = "/home/josh/Documents/UCT/Thesis/Code/kitti/full_run/drive_27/no_integration/";
no_merge_dir = "/home/josh/Documents/UCT/Thesis/Code/kitti/full_run/drive_27/no_merge/";
interactive_dir = "/home/josh/Documents/UCT/Thesis/Code/kitti/full_run/drive_27/no_merge/interactive/";
full_interactive_dir = "/home/josh/Documents/UCT/Thesis/Code/kitti/full_run/drive_27/full_interactive/";
bb_dir = "/home/josh/Documents/UCT/Thesis/Code/kitti/full_run/drive_27/bb/";
tens_dir = "/home/josh/Documents/UCT/Thesis/Code/kitti/full_run/drive_27/pure_tensorflow/";

% 2_sampling
% frame_integration_1 = "/home/josh/Documents/UCT/Thesis/Code/kitti/full_run/drive_27/2_sampling/1_f_1_b/";
% frame_integration_2 = "/home/josh/Documents/UCT/Thesis/Code/kitti/full_run/drive_27/2_sampling/2_b/";
% segmentation_dir = "/home/josh/Documents/UCT/Thesis/Code/kitti/full_run/drive_27/2_sampling/no_integration/";
% no_merge_dir = "/home/josh/Documents/UCT/Thesis/Code/kitti/full_run/drive_27/2_sampling/no_merge/";
% interactive_dir = "/home/josh/Documents/UCT/Thesis/Code/kitti/full_run/drive_27/2_sampling/interactive/";

% 3_sampling
% frame_integration_1 = "/home/josh/Documents/UCT/Thesis/Code/kitti/full_run/drive_27/3_sampling/1_f_1_b/";
% frame_integration_2 = "/home/josh/Documents/UCT/Thesis/Code/kitti/full_run/drive_27/3_sampling/2_b/";
% segmentation_dir = "/home/josh/Documents/UCT/Thesis/Code/kitti/full_run/drive_27/3_sampling/no_integration/";
% no_merge_dir = "/home/josh/Documents/UCT/Thesis/Code/kitti/full_run/drive_27/3_sampling/no_merge/";
% interactive_dir = "/home/josh/Documents/UCT/Thesis/Code/kitti/full_run/drive_27/3_sampling/interactive/";

% 4_sampling
% frame_integration_1 = "/home/josh/Documents/UCT/Thesis/Code/kitti/full_run/drive_27/4_sampling/1_f_1_b/";
% frame_integration_2 = "/home/josh/Documents/UCT/Thesis/Code/kitti/full_run/drive_27/4_sampling/2_b/";
% segmentation_dir = "/home/josh/Documents/UCT/Thesis/Code/kitti/full_run/drive_27/4_sampling/no_integration/";
% no_merge_dir = "/home/josh/Documents/UCT/Thesis/Code/kitti/full_run/drive_27/4_sampling/no_merge/";
% interactive_dir = "/home/josh/Documents/UCT/Thesis/Code/kitti/full_run/drive_27/4_sampling/interactive/";

% 5_sampling
% frame_integration_1 = "/home/josh/Documents/UCT/Thesis/Code/kitti/full_run/drive_27/5_sampling/1_f_1_b/";
% frame_integration_2 = "/home/josh/Documents/UCT/Thesis/Code/kitti/full_run/drive_27/5_sampling/2_b/";
% segmentation_dir = "/home/josh/Documents/UCT/Thesis/Code/kitti/full_run/drive_27/5_sampling/no_integration/";
% no_merge_dir = "/home/josh/Documents/UCT/Thesis/Code/kitti/full_run/drive_27/5_sampling/no_merge/";
% interactive_dir = "/home/josh/Documents/UCT/Thesis/Code/kitti/full_run/drive_27/5_sampling/interactive/";


% % % % % % % % % % % % %% % % % % % % % % % % % % % % % %

% 2011_10_03_drive_27
% ros - 'Training'
% gt_dir = "/home/josh/Documents/UCT/Thesis/Datasets/ground_truth_segmentation/ros_offline/KITTI_SEMANTIC/Training_00/GT/";
% base_dir = "/home/josh/Documents/UCT/Thesis/Datasets/ground_truth_segmentation/ros_offline/KITTI_SEMANTIC/Training_00/RGB/";
% frame_integration_1 = "/home/josh/Documents/UCT/Thesis/Code/kitti/full_run/10_03_drive_27/1_f_1_b/no_merge/";
% frame_integration_2 = "/home/josh/Documents/UCT/Thesis/Code/kitti/full_run/10_03_drive_27/2_b/no_merge/";
% segmentation_dir = "/home/josh/Documents/UCT/Thesis/Code/kitti/full_run/10_03_drive_27/no_integration/";
% no_merge_dir = "/home/josh/Documents/UCT/Thesis/Code/kitti/full_run/10_03_drive_27/no_merge/";
% interactive_dir = "/home/josh/Documents/UCT/Thesis/Code/kitti/full_run/10_03_drive_27/no_merge/interactive/";


files = dir(gt_dir);
num_files = size(files);

frame_int_jaccard_1 = zeros(size(num_files, 1)-2, 1);
frame_int_jaccard_2 = zeros(size(num_files, 1)-2, 1);
no_integration_jaccard = zeros(size(num_files, 1)-2, 1);
no_merge_jaccard = zeros(size(num_files, 1)-2, 1);
interactive_jaccard = zeros(size(num_files, 1)-2, 1);
full_interactive_jaccard = zeros(size(num_files, 1)-2, 1);
bb_jaccard = zeros(size(num_files, 1)-2, 1);
tens_jaccard = zeros(size(num_files, 1)-2, 1);

for i = 1:num_files
    frame = files(i).name;
    if (strcmp(frame, '.') || strcmp(frame, '..'))
        continue;
    else
        frame = str2double(frame(1:end-4));
    end

    gt_img = imread(sprintf('%s%06d.png', gt_dir, frame));
    img = imread(sprintf('%s%06d.png', base_dir, frame));

    % read segmentation masks from disk
    try
        frame_1_mask = load(sprintf('%s%d_mask.mat', frame_integration_1, frame));
        frame_2_mask = load(sprintf('%s%d_mask.mat', frame_integration_2, frame));
        no_integration_mask = load(sprintf('%s%d_mask.mat', segmentation_dir, frame));
        no_merge_mask = load(sprintf('%s%d_mask.mat', no_merge_dir, frame));
        interactive_mask = load(sprintf('%s%d_interactive_mask.mat', interactive_dir, frame));
        full_interactive_mask = load(sprintf('%s%d_interactive_mask.mat', full_interactive_dir, frame));
        bb_mask = load(sprintf('%s%d_mask.mat', bb_dir, frame));
        tens_mask = load(sprintf('%s%d_mask.mat', tens_dir, frame));
    catch
        continue;
    end

    % convert masks to logical arrays
    frame_1_mask = logical(frame_1_mask.mask);
    frame_2_mask = logical(frame_2_mask.mask);
    no_integration_mask = logical(no_integration_mask.mask);
    no_merge_mask = logical(no_merge_mask.mask);
    interactive_mask = logical(interactive_mask.mask);
    full_interactive_mask = logical(full_interactive_mask.mask);
    bb_mask = logical(bb_mask.mask);
    tens_mask = logical(tens_mask.mask);

    r = gt_img(:,:,1);
    g = gt_img(:,:,2);
    b = gt_img(:,:,3);

    % find import things in ground truth masks
    car_mask = ((r == 64) & (g == 0) & (b == 128));
    person_mask = ((r == 64) & (g == 64) & (b == 0));

    % combine things of importance
    full_mask = car_mask | person_mask;

    % compute IoU (jaccard)
    frame_int_jaccard_1(i) = jaccard(full_mask, frame_1_mask);
    frame_int_jaccard_2(i) = jaccard(full_mask, frame_2_mask);
    no_integration_jaccard(i) = jaccard(full_mask, no_integration_mask);
    no_merge_jaccard(i) = jaccard(full_mask, no_merge_mask);
    interactive_jaccard(i) = jaccard(full_mask, interactive_mask);
    full_interactive_jaccard(i) = jaccard(full_mask, full_interactive_mask);
    bb_jaccard(i) = jaccard(full_mask, bb_mask);
    tens_jaccard(i) = jaccard(full_mask, tens_mask);
end

jaccards = {frame_int_jaccard_1, frame_int_jaccard_2, no_integration_jaccard, no_merge_jaccard, interactive_jaccard, full_interactive_jaccard, bb_jaccard, tens_jaccard};

% calculate mean IoU
for i = 1:size(jaccards, 2)
    jaccards{i} = rmmissing(jaccards{i});
    disp(mean(jaccards{i}))
end

% masked = bsxfun(@times, gt_img, cast(car_mask,class(gt_img)));

% binary_mask = (masked ~= 0);
% imshow(full_mask)

% imshow(car_mask);
