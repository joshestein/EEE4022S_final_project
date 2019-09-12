function calib = load_odometry(filename)
    % reads ground-truth odometry data
    % calib is an m-by-12, where each row represents the pose at some particular frame
    % read 'poses' under Kitti odometry devkit readme.txt for more info

% open file
fid = fopen(filename,'r');

if fid<0
  calib = [];
  return;
end

% read maximum of 5000 poses
for pose = 1:5000
    Tr_matrix = zeros(4, 4);
    for m = 1:3
        for n = 1:4
            % read single float
            [val, success] = fscanf(fid, '%f', 1);
            if success
                Tr_matrix(m, n) = val;
            else
                Tr_matrix = [];
            end
        end
    end
    Tr_matrix(4,:) = [0, 0, 0, 1];
    % break when no new values
    if isempty(Tr_matrix)
        break;
    end
    calib{pose} = Tr_matrix;
end