import skimage.segmentation
import scipy
from matplotlib import pyplot as plt

img = scipy.misc.imread("lenna.ppm")

segment_mask_50 = skimage.segmentation.felzenszwalb(img, scale=50)
segment_mask_100 = skimage.segmentation.felzenszwalb(img, scale=100)
segment_mask_300 = skimage.segmentation.felzenszwalb(img, scale=300)
segment_mask_1000 = skimage.segmentation.felzenszwalb(img, scale=1000)

fig, axs = plt.subplots(4)
axs[0].imshow(segment_mask_50)
axs[0].set_xlabel("k=50")
axs[1].imshow(segment_mask_100)
axs[1].set_xlabel("k=100")
axs[2].imshow(segment_mask_300)
axs[2].set_xlabel("k=300")
axs[3].imshow(segment_mask_1000)
axs[3].set_xlabel("k=1000")

plt.show()
