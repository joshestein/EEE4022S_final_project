SDK_DIR = '../../Datasets/robotcar-dataset-sdk-2.1.1';
path(path, [SDK_DIR '/matlab/'])

image_points_w_colour = 'image_points.jpg';
image_points_white = 'white_image_points.jpg';
orig_img = 1400075815389497;

% Parameters
% block size = nxn block used to search for pixels
% threshold = number of pixels per nxn block before drawing a rec
block_size = 16;
threshold = 90;
% % % % % % % % %

img = imread('white_image_points.jpg');
orig_img = LoadImage('../../Datasets/2014-05-14-13-53-47/stereo/centre/', orig_img);
% orig_img = imread('original_image.jpg');


[rows, cols, channels] = size(orig_img);
% imshow(orig_img);
hold on;
axis on;

% ax.YDir = 'normal';
% for row = 1:block_size:rows
%     line([1, cols], [row, row], 'Color', 'r');
% end
% for col = 1:block_size:cols
%     line([col, col], [1, row], 'Color', 'b');
% end

% rect_axes = axes('Position', [0 1280 0 960]);
% set(rect_axes, 'YDir', 'reverse');
% for row = 1:block_size:rows-block_size
%     for col = 1:block_size:cols-block_size
%         num_pixels = find(img(row:row+block_size, col:col+block_size) ~= 0);
%         % disp(['Num pixels: ' num2str(size(num_pixels, 1))]);
%         if (size(num_pixels, 1) > threshold)
%             % axis([0 1280 960 0])
%             rectangle('Position', [col row block_size block_size], 'EdgeColor', 'g');
%             % disp(['GT at: ' num2str(row) ', ' num2str(col)]);
%             % rect_axes;
%         end
%     end
% end

block_starts = [];
% mask = false(size(orig_img, 1), size(orig_img, 2));
grab_mask = false(size(orig_img, 1), size(orig_img, 2));
roi = grab_mask;
roi(10:end-10,10:end-10,:) = true;

L = superpixels(orig_img, 500);

for col = 1:block_size:cols
    % reset mask for grabcut
    mask = false(size(orig_img, 1), size(orig_img, 2));
    % col + block_size = 1 + block points
    % so we need to subtract one to ensure we are only searching for exactly block number of points.
    % e.g block size = 16
    % 1:(1+16) = searching 17 points
    % So explicitely subtract 1.
    col_end = col + block_size - 1;
    first_row = true;
    first_row_pos = rows;
    % start from the bottom to detect points lower down
    % height is an indication that an object is present
    for row = rows:-(block_size):1

        % same explanation as col_end
        % but here we are looping backwards, so the end point is correct
        % the start point is always 1 too small
        % e.g. (16-16):16
        % should be (16-15):16
        start_row = row-block_size+1;

        % find pixels ~= 0
        num_pixels = find(img(start_row:row, col:col_end) ~= 0);
        if (size(num_pixels, 1) > threshold)
            % prevent distant rows (rows much further down) from always letting higher row rectangle displays
            % 
            % threshold of 3*block_size is arbitrary
            if (~first_row && (first_row_pos - row) < 3*block_size) 
                %rectangle('Position', [col start_row block_size block_size], 'EdgeColor', 'r');
                mask(start_row:row, col:col_end) = true;
                fmask = mask;
                bmask = ~mask;
                BW = grabcut(orig_img, L, roi, mask, bmask);
                grab_mask = grab_mask + BW;
                block_starts = [block_starts ; col start_row];
            end
            first_row_pos = row;
            first_row = false;
        end
    end
end

imshow(grab_mask)

grid on;
% find countours starting from highest density lidar points
% this guy is pretty slow
% bw = activecontour(orig_img, mask, 200);
% imshow(orig_img);

% generate labels for grabcut
% L = superpixels(orig_img, 700);

% generate a mask from superpixels
% bw = boundarymask(L);

% bwboundaries just generates boundaries (exterior of objects) in a grayscale image
% [B, N] = bwboundaries(mask, 'noholes');
% plot bwboundaries
% for k = 1:length(B)
%     boundary = B{k};
%     plot(boundary(:,2), boundary(:,1), 'w', 'Linewidth', 2);
% end

% grab = grabcut(orig_img, L, mask);
% imshow(grab);


% draw original mask an detected contours
% contour(mask, 1, 'g', 'LineWidth', 4);
% contour(bw, 1, 'r', 'LineWidth', 4);