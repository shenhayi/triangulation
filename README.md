# triangulation
triangulation
## Working Principal
### Decoding

### Triangulation
- 2D position and depth information for each object are decoded.
- Point cloud with respect to the camera could then be transferred to 3D with camera intrinsics.
- Transfer point cloud into world frame with camera position in world frame.
- Define bounding boxes and positions of detected objects. 

### Map Generation

## Run Triangulation
install triangulation,change "class_labels_path" in triangulation/cfg/triangulation.yaml
```
class_labels_path: PATH_TO/triangulation/class_labels/coco.txt
```
install target labelling
```
git clone https://github.com/Zhefan-Xu/target_labelling.git
```
run yolo
```
rosbag play -l exploration_yolo.bag 
roslaunch yolov7_seg_ros yolov7.launch 
```
run triangulation
```
roslaunch triangulation triangulation.launch
```
