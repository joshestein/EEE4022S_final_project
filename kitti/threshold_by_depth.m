function masks = threshold_by_depth(img, velo_img, velo, opt_depth_step)

% builds an array of masks based on depth
% assumes a jet colourmap
%
% Parameters:
% img               | image to plot on
% velo_img          | LiDAR point cloud projected onto img
% velo              | LiDAR point cloud
% opt_depth_step    | the amount to step for each iteration. Defaults to 8.
% 
% Output:
% masks is an n dimensional matrix, where each dimension is a mask corresponding to the size of the image, storing pixels of objects at particular depths

if nargin > 3
    depth_step = opt_depth_step;
    disp(depth_step);
else
    depth_step = 8;
end

masks = zeros(size(img, 1), size(img, 2), round(size(jet, 1)/depth_step));

% either loop through all the depths (pre-computed), scan image for each depth
% or loop through the image, compute depths and assign to appropriate mask
% looping through the image (once) should be quicker than looping through each time for each depth step
% 64 rows of jet colourmap
% can definitely do without a loop
for i = 1:size(velo_img, 1)
    col_idx = round(64*5/velo(i,1));
    % place into nearest bin
    bin = round(col_idx*(1/depth_step));
    row = round(velo_img(i, 2));
    col = round(velo_img(i, 1));

    masks(row, col, bin) = 1;
end