# STAM (Modified for providing Visual Odometry in GraphSlam)
Simple Tracking and Mapping - STAM

This is the modified version of STAM implementation for providing visual odometry and pose estimations for Graph-Based SLAM.

### TODO ###

* Quick summary
* Version
* [Learn Markdown](https://bitbucket.org/tutorials/markdowndemo)

### Dependencies ###

* OpenCV 2.4.x
* LAPACK (https://pheiter.wordpress.com/2012/09/04/howto-installing-lapack-and-blas-on-mac-os/)


### Building STAM Linux ###

````
git clone -b graphslam_mod https://github.com/eaa3/STAM.git

cd STAM
   
mkdir build && cd build

cmake ..

make -j 
````
