SDK_DIR = '../../Datasets/robotcar-dataset-sdk-2.1.1';
DATA = '../../Datasets/robotcar-dataset';
path(path, [SDK_DIR '/matlab/'])

IMAGE_DIR = [DATA '/stereo/centre/'];
LIDAR_DIR = [DATA '/ldmrs/'];
INS_FILE = [DATA '/gps/ins.csv'];
EXTRINSIC_DIR = [SDK_DIR '/extrinsics/'];
MODELS_DIR = [SDK_DIR '/models/'];

start_timestamp = 1418381799701502;
end_timestamp = 1418381799826501;

% img = LoadImage(IMAGE_DIR, start_timestamp);
% imshow(img);
% BuildPointcloud(LIDAR_DIR, INS_FILE, EXTRINSIC_DIR, start_timestamp, end_timestamp);

ProjectLaserIntoCamera(IMAGE_DIR, LIDAR_DIR, INS_FILE, MODELS_DIR, EXTRINSIC_DIR, start_timestamp);