from skimage.color import rgb2gray
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt

image = plt.imread('1.jpeg')/255
image_reshape = image.reshape(image.shape[0]*image.shape[1], image.shape[2])
clusters = int(input("Enter number of clusters:\n"))

kmeans = KMeans(n_clusters=clusters, random_state=0).fit(image_reshape)
result = kmeans.cluster_centers_[kmeans.labels_]

cluster_pic = result.reshape(image.shape[0], image.shape[1], image.shape[2])
plt.imshow(cluster_pic)
plt.show()