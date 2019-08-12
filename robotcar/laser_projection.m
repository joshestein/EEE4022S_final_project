SDK_DIR = '../../Datasets/robotcar-dataset-sdk-2.1.1';
% sample_DATA = '../../Datasets/robotcar-dataset';
% sample_start_timestamp = 1418381799701502;
% sample_end_timestamp = 1418381799826501;

% 2014_DATA = '../../Datasets/2014-05-14-13-53-47';
% 2014_start_timestamp = 1400075815389497;
% 5 frames later...
% 2014_end_timestamp = 1400075815701972;

DATA = '/home/josh/Documents/UCT/Thesis/Datasets/2015-11-10-10-32-52';
%start_timestamp = 1447151694756261;
end_timestamp = start_timestamp+10e7;
% start_timestamp = 1447151634694061;
start_timestamp =  1447151671318873;
% next_ frame: 1447151671443870
% next_next_frame: 1447151671506369
% 5 frames later: 1447151671631368
% another: 1447151685006313
path(path, [SDK_DIR '/matlab/'])

IMAGE_DIR = [DATA '/stereo/centre/'];
LIDAR_DIR = [DATA '/lms_front/'];
INS_FILE = [DATA '/gps/ins.csv'];
EXTRINSIC_DIR = [SDK_DIR '/extrinsics/'];
MODELS_DIR = [SDK_DIR '/models/'];

% img = LoadImage(IMAGE_DIR, start_timestamp);
% imshow(img);
% BuildPointcloud(LIDAR_DIR, INS_FILE, EXTRINSIC_DIR, start_timestamp);

ProjectLaserIntoCamera(IMAGE_DIR, LIDAR_DIR, INS_FILE, MODELS_DIR, EXTRINSIC_DIR, start_timestamp);