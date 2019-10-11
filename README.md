# EEE4022S Final year project

The MATLAB can be found in the 'kitti' folder.

Use the master branch to segment a single image (non-interactively).

Switch to branch 'interactive' to segment a single image interactively. In the interactive window, you can left-click on a polygon to remove it, right-click on a polygon to remove all other polygons (except that one) or press `a` to switch to an append mode, wherein you can add new polygons. When adding new polygons, you can press `esc` to start over. To quit the append mode, press `esc` or `enter`.

Switch to branch 'full_interactive' to segment multiple images, either non-interactively (i.e. comment out `polygons = interactive(img, polygons)` - line 511) or interactively (leave as is).

Branch 'bb' is used to refine TensorFlow bounding boxes. Bounding boxes are generated using code from: https://github.com/tensorflow/models/blob/master/research/object_detection/object_detection_tutorial.ipynb, with the backbone changed to 'faster_rcnn_resnet101_kitti' from: https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md
