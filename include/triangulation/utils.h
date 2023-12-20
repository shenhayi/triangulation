#ifndef UTILS_H
#define UTILS_H

struct vertex{
	double xmin, ymin, zmin;
    double xmax, ymax, zmax;
    double idx;
    vertex(){
        xmin = 0; ymin = 0; zmin = 0; xmax = 0; ymax = 0; zmax = 0; idx = 0;
    }
    vertex(double xmin_, double ymin_, double zmin_, double xmax_, double ymax_, double zmax_, double idx_){
        xmin = xmin_; ymin = ymin_; zmin = zmin_; xmax = xmax_; ymax = ymax_; zmax = zmax_; idx = idx_;
    }
};

struct object{
    double x, y, z;
    double xsize, ysize, zsize;
    double label;
    object(){
        x = 0; y= 0; z = 0;
        xsize = 0; ysize = 0; zsize = 0;
        label = 0;
    }
    object(double x_, double y_, double z_, double xsize_, double ysize_, double zsize_, double label_){
        x = x_; y= y_; z = z_;
        xsize = xsize_; ysize = ysize_; zsize = zsize_;
        label = label_;
    }
};

// inline double getDistance(pose p1, pose p2){

//     return sqrt(pow((p1.x - p2.x),2) + pow((p1.y - p2.y),2) + pow((p1.z - p2.z),2));	

// }



#endif