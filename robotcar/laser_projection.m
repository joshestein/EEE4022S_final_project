SDK_DIR = '../../Datasets/robotcar-dataset-sdk-2.1.1';
DATA = '../../Datasets/robotcar-dataset';
path(path, [SDK_DIR '/matlab/'])

IMAGE_DIR = [DATA '/stereo/centre/'];
LIDAR_DIR = [DATA '/ldmrs/'];
INS_FILE = [DATA '/gps/ins.csv'];
EXTRINSIC_DIR = [SDK_DIR '/extrinsics/'];
MODELS_DIR = [SDK_DIR '/models/'];

timestamp = 1418381799701502;

% img = LoadImage(IMAGE_DIR, timestamp);
% BuildPointcloud(LIDAR_DIR, INS_FILE, EXTRINSIC_DIR);

ProjectLaserIntoCamera(IMAGE_DIR, LIDAR_DIR, INS_FILE, MODELS_DIR, EXTRINSIC_DIR, timestamp);