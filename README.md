# Triangulation

Triangulation part for segementation.

## Working Principal

### Decoding

* subscribe to the target topic and get the original data which includes label, position and depth.
* divide the data into n-channel with each channel represents one object detected.
* fill the n-channel mask with data.

### Triangulation

- 2D position and depth information for each object are decoded.
- Point cloud with respect to the camera could then be transferred to 3D with camera intrinsics.
- Transfer point cloud into world frame with camera position in world frame.
- Define bounding boxes and positions of detected objects.

### Map Generation

* store the boundingboxs of the all the objectes detected.
* erase the repeated objects by checking the label and COG.
* visulize all the boundingboxs

## Run Triangulation

Clone the package and build it

```
cd PATH_TO/catkin_ws/src
git clone https://github.com/shenhayi/triangulation.git
cd ..
catkin_make
source devel/setup.bash
```

Install triangulation,change "class_labels_path" in triangulation/cfg/triangulation.yaml

```
class_labels_path: PATH_TO/triangulation/class_labels/coco.txt
```

Install target labelling

```
git clone https://github.com/Zhefan-Xu/target_labelling.git
```

Play your data and run segementation

```
rosbag play -l exploration_yolo.bag 
roslaunch yolov7_seg_ros yolov7.launch 
```

Run triangulation

```
roslaunch triangulation triangulation.launch
```
