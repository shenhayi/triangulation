# triangulation
triangulation
# Run Triangulation
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
