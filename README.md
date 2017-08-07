# STAM (Visual Odometry for GraphSlam)
Simple Tracking and Mapping - STAM
This is the modified version of STAM implementation for providing visual odometry and pose estimations for Graph-Based SLAM

### TODO ###

* Quick summary
* Version
* [Learn Markdown](https://bitbucket.org/tutorials/markdowndemo)

### Dependencies ###

* OpenCV 2.4.x
* cvsba (https://www.uco.es/investiga/grupos/ava/node/39)
* LAPACK (https://pheiter.wordpress.com/2012/09/04/howto-installing-lapack-and-blas-on-mac-os/)


### Installing dependencies on OSX ###

* opencv 2.4.x
  * ``brew tap homebrew/science``
  * `` brew install opencv ``
* f2c
  * ``brew install --HEAD homebrew/boneyard/f2c``
* cvsba
  * Download, make and install csvba: https://www.uco.es/investiga/grupos/ava/node/39
  
### Installing dependencies on Ubuntu ###

* cvsba
  * Follow UNIX instructions at https://www.uco.es/investiga/grupos/ava/node/39



### Building STAM Linux/OSX ###

````
git clone -b graphslam_mod https://github.com/eaa3/STAM.git

cd STAM
   
mkdir build && cd build

cmake ..

make -j 
````
