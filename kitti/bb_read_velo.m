function [velo, velo_img] = bb_read_velo(filename, transform_matrix)
    % read velo points and project them to image
    % inputs:
    % transform matrix the matrix used to transform velo points to image
    % % % % % % % % %
    % outputs:
    % velo is the set of velodyne points (unprojected)
    % velo_img is the set of projected points

  global base_dir;
  global img;

  [fid, message] = fopen(sprintf('%s/velodyne_points/data/%010d.bin',base_dir,filename),'rb');
  velo = [];
  if (fid < 0)
    error('Failed to read "%s" error "%s"', fid, message);
    return;
  else
    velo = fread(fid,[4 inf],'single')';
  end

  if (isempty(velo))
    disp("Failed to read Velodyne data.");
    return;
  end
  % velo = velo(1:10:end,:); % remove every 5th point for display speed
  fclose(fid);

  % remove all points behind image plane (approximation
  idx = velo(:,1)<0;
  velo(idx,:) = [];

  % remove points that have a height of ~ 0.2 m
  % thresh of 1 is conservative
  % negative because velo is centered at 0 on each of its axes
  % that is, half the points are above z = 0 and half below
  height_thresh = -1.3;
  idx = velo(:,3) < height_thresh;
  velo(idx,:) = [];

  % project to image plane (exclude luminance)
  velo_img = project(velo(:,1:3), transform_matrix);

  % remove all points outside of image
  outside_idx = (round(velo_img(:,1)) > size(img,2) | round(velo_img(:, 1)) <= 0 | round(velo_img(:,2)) > size(img,1) | round(velo_img(:, 2)) <= 0);

  velo_img(outside_idx, :) = [];
  velo(outside_idx,:) = [];
end
