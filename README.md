# triangulation
triangulation
# Run Triangulation
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
