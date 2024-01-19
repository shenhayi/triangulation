# triangulation
triangulation
# Run Triangulation
run yolo
```
git clone https://github.com/Zhefan-Xu/target_labelling.git
rosbag play -l exploration_yolo.bag 
roslaunch yolov7_seg_ros yolov7.launch 
```
run triangulation
```
roslaunch triangulation triangulation.launch
```
