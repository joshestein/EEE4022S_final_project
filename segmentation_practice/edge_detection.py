from skimage.color import rgb2gray
import numpy as np 
import matplotlib.pyplot as plt
from scipy import ndimage

image = plt.imread('pepper.ppm')
gray_img = rgb2gray(image)

sobel_horizontal = np.array([[1, 2, 1], [0, 0, 0], [-1, -2, -1]])
sobel_vertical = np.array([[-1, 0, 1], [-2, 0, 2], [-1, 0, 1]])
laplace = np.array([[1, 1, 1], [1, -8, 1], [1, 1, 1]])

out_h = ndimage.convolve(gray_img, sobel_horizontal, mode="reflect")
out_v = ndimage.convolve(gray_img, sobel_vertical, mode="reflect")
out_l = ndimage.convolve(gray_img, laplace, mode="reflect")

fig, axs = plt.subplots(1, 3)
axs[0].imshow(out_h, cmap='gray')
axs[0].set_title("Horizontal")

axs[1].imshow(out_v, cmap='gray')
axs[1].set_title("Vertical")

axs[2].imshow(out_l, cmap='gray')
axs[2].set_title("Laplace")
plt.show()
