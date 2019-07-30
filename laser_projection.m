SDK_DIR = '../../Datasets/robotcar-dataset-sdk-2.1.1';
DATA = '../../Datasets/robotcar-dataset';

IMAGE_DIR = [DATA '/stereo/centre/'];
LIDAR_DIR = [DATA '/ldmrs/'];
INS_FILE = [DATA '/gps/ins.csv'];
EXTRINSIC_DIR = [SDK_DIR '/extrinsics/'];

test_img = '1418381798076682';
path(path, SDK_DIR);
path(path, DATA);

img = LoadImage(IMAGE_DIR, test_img);
%imshow(img)

BuildPointcloud(LIDAR_DIR, INS_FILE, EXTRINSIC_DIR);