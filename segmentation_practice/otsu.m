img = imread('pepper.ppm');
figure;
imshow(img);
img_gray = rgb2gray(img);
figure;
imshow(img_gray);
level = graythresh(img_gray);
bw = imbinarize(img_gray, level);
figure;
imshow(bw);
figure;
subplot(1,3,1);
imshow(img);
title('Original image')
subplot(1,3,2);
imshow(img_gray);
title('Gray-scale image')
subplot(1,3,3);
imshow(bw);
title("Thresholding using Otsu's method");
