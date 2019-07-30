# Way to segment image based on colour thresholds
# Fast and simple
# Works well for high-contrast images

from skimage.color import rgb2gray
import matplotlib.pyplot as plt

image = plt.imread('1.jpeg')
gray_img = rgb2gray(image)

gray_r = gray_img.reshape(gray_img.shape[0]*gray_img.shape[1])
mean = gray_r.mean()
print(mean)
thresholds = int(input("Enter the number of thresholds:\n"))

for i in range(gray_r.shape[0]):
    for j in range(thresholds, 0, -1):
        if (gray_r[i] > (1-1/j)):
            gray_r[i] = (1-1/j)
            break
        if (j == 1):
            gray_r[i] = 0

gray_img = gray_r.reshape(gray_img.shape[0], gray_img.shape[1])
plt.imshow(gray_img, cmap='gray')
plt.show()