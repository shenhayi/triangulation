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



#endif